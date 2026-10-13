"""Mission transitions without ROS or hardware side effects."""
from collections import deque
import math


class MissionState:
    def __init__(self):
        self.stage = 'manual'
        self.has_ball = False
        self.rearm_required = False

    def step(self, automatic, localization_ready, ball_ready, near_ball,
             camera_ready, camera_lost, goal_complete):
        if not automatic:
            if self.stage == 'complete':
                self.has_ball = False
            self.stage = 'manual'
            self.rearm_required = False
        elif self.rearm_required:
            self.stage = 'wait_manual_rearm'
        elif self.stage == 'complete':
            pass
        elif self.has_ball:
            self.stage = 'complete' if goal_complete else 'deliver_goal'
        elif self.stage == 'visual_acquire' and not camera_lost:
            pass
        elif localization_ready and ball_ready:
            self.stage = 'visual_acquire' if near_ball and camera_ready else 'approach_ball'
        else:
            self.stage = 'wait_global'
        return self.stage

    def confirm_capture(self, automatic, drone_only=False):
        allowed = ('approach_ball', 'visual_acquire', 'wait_global') if drone_only else ('visual_acquire',)
        if not automatic or self.rearm_required or self.stage not in allowed:
            return False
        self.has_ball = True
        self.stage = 'deliver_goal'
        return True

    def stream_changed(self):
        self.rearm_required = True
        self.stage = 'wait_manual_rearm'

    def lose_ball(self):
        self.has_ball = False
        self.stage = 'wait_global'


class PossessionGuard:
    """Only new validated drone frames refresh possession; no camera-visibility assumption."""
    def __init__(self):
        self.last_held = None
        self.negative_since = self.negative_at = None

    def start(self, now):
        self.last_held = now
        self.negative_since = self.negative_at = None

    def observe(self, possession, received):
        if possession == 'held':
            self.last_held = received
            self.negative_since = self.negative_at = None
        elif possession in ('free', 'candidate'):
            if self.negative_at is None or received-self.negative_at > .5:
                self.negative_since = received
            self.negative_at = received
        else:
            self.negative_since = self.negative_at = None

    def loss_reason(self, now, confirm_sec, timeout_sec):
        if (self.negative_at is not None and 0 <= now-self.negative_at <= .5 and
                self.negative_at-self.negative_since >= confirm_sec):
            return 'drone_possession_not_held'
        if self.last_held is not None and now-self.last_held >= timeout_sec:
            return 'drone_possession_timeout'
        return None


def forward_floor(values, minimum):
    """Maintain mean forward demand, not a measured boat speed; retain steering where possible."""
    result = [max(0., min(100., float(v))) for v in values]
    deficit = max(0., 2*minimum-sum(result))
    add = deficit/2
    result = [min(100., v+add) for v in result]
    remaining = max(0., 2*minimum-sum(result))
    index = 0 if result[0] <= result[1] else 1
    result[index] = min(100., result[index]+remaining)
    return result


class HeldVote:
    def __init__(self):
        self.samples = deque(maxlen=5000)

    def clear(self):
        self.samples.clear()

    def observe(self, received, held, window):
        if self.samples and (received <= self.samples[-1][0] or received-self.samples[-1][0] > .5):
            self.clear()
        self.samples.append((received, bool(held)))
        while self.samples and received-self.samples[0][0] > window:
            self.samples.popleft()

    def evidence(self, now):
        count = len(self.samples)
        return dict(count=count, held_ratio=sum(s[1] for s in self.samples)/count if count else 0.,
                    duration_sec=self.samples[-1][0]-self.samples[0][0] if count else 0.,
                    fresh=bool(count and 0 <= now-self.samples[-1][0] <= .5))


class StuckRecovery:
    """Fresh measured positions only; bounded pivot, not proof of a field boundary."""
    def __init__(self):
        self.anchor = None
        self.since = self.last_sample = None
        self.count = 0
        self.active_since = None
        self.last_tick = None
        self.turned = 0.
        self.cooldown_until = 0.

    def reset_window(self):
        self.anchor = self.since = self.last_sample = None
        self.count = 0

    def cancel(self):
        self.active_since = None
        self.reset_window()

    def step(self, now, stamp, xy, forward, allowed, yaw_rate, c):
        dt = max(0., min(.2, now-self.last_tick)) if self.last_tick is not None else 0.
        self.last_tick = now
        if not allowed or stamp is None or xy is None or not 0 <= now-stamp <= c['stuck_position_max_age_sec']:
            self.cancel()
            return False
        if self.active_since is not None:
            self.turned += abs(yaw_rate)*dt
            if now-self.active_since >= c['stuck_turn_duration_sec'] or self.turned >= c['stuck_turn_angle_rad']:
                self.active_since = None
                self.cooldown_until = now+c['stuck_cooldown_sec']
                self.reset_window()
                return False
            return True
        if not forward or now < self.cooldown_until:
            self.reset_window()
            return False
        if stamp == self.last_sample:
            return False
        if (self.anchor is None or self.last_sample is None or stamp < self.last_sample or
                stamp-self.last_sample > c['stuck_position_max_age_sec'] or
                math.dist(xy, self.anchor) > c['stuck_position_tolerance_m']):
            self.anchor, self.since, self.count = list(xy), stamp, 0
        self.last_sample = stamp
        self.count += 1
        if self.count >= 3 and stamp-self.since > c['stuck_detection_sec']:
            self.active_since, self.turned = now, 0.
            self.reset_window()
            return True
        return False
