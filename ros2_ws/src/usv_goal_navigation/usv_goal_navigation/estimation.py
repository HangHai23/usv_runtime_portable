"""Map memory and delayed-observation replay; no ROS dependency."""
import math
from collections import deque
from .control import wrap_angle


def point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('expected 2D point')
    if not all(type(v) in (int, float) and math.isfinite(v) for v in value):
        raise ValueError('invalid coordinates')
    return list(map(float, value))


class MapMemory:
    def __init__(self, smoothing_gain=0.0, tolerance_m=0.5):
        self.smoothing_gain = smoothing_gain
        self.tolerance_m = tolerance_m
        self.coord = None
        self.size = None
        self.goals = {}

    def ingest(self, observation):
        coord = observation['coord']
        if not isinstance(coord, str) or not coord:
            raise ValueError('invalid coordinate identity')
        if self.coord is not None and coord != self.coord:
            raise ValueError('coordinate_reset')
        if observation.get('field_valid') is not True:
            return False
        size = point(observation['size_m']) if observation.get('size_m') is not None else None
        if size and min(size) <= 0:
            raise ValueError('invalid field size')
        additions = {}
        for goal in observation.get('goals', []):
            if goal['side'] not in ('left', 'right') or len(goal['ends_m']) != 2:
                raise ValueError('invalid goal')
            ends = [point(p) for p in goal['ends_m']]
            if math.dist(*ends) <= 0:
                raise ValueError('zero length goal')
            additions[goal['side']] = ends
        changed = self.coord is None
        self.coord = coord
        if self.size is None and size:
            self.size = size
            changed = True
        elif size and self.smoothing_gain and max(abs(a-b) for a,b in zip(size,self.size)) <= self.tolerance_m:
            updated = [a+self.smoothing_gain*(b-a) for a,b in zip(self.size,size)]
            if max(abs(a-b) for a,b in zip(updated,self.size)) > 1e-6:
                self.size = updated
                changed = True
        for side, ends in additions.items():
            if side not in self.goals:
                self.goals[side] = ends
                changed = True
            elif self.smoothing_gain and max(math.dist(a,b) for a,b in zip(self.goals[side],ends)) <= self.tolerance_m:
                updated = [[a+self.smoothing_gain*(b-a) for a,b in zip(old,new)]
                           for old,new in zip(self.goals[side],ends)]
                if max(math.dist(a,b) for a,b in zip(updated,self.goals[side])) > 1e-6:
                    self.goals[side] = updated
                    changed = True
        return changed

    def snapshot(self):
        w, h = self.size or (0, 0)
        return dict(coord=self.coord, size_m=self.size, goals=self.goals,
                    boundary=[[0, 0], [w, 0], [w, h], [0, h]] if self.size else [])


def propagate(state, dt, yaw_rate, speed_target, tau):
    x, y, heading, speed = state
    next_speed = speed_target + (speed - speed_target) * math.exp(-dt / tau)
    travel = speed_target * dt + (speed - speed_target) * tau * (1 - math.exp(-dt / tau))
    mid_heading = heading + 0.5 * yaw_rate * dt
    return [x + travel * math.cos(mid_heading), y + travel * math.sin(mid_heading),
            wrap_angle(heading + yaw_rate * dt), next_speed]


class VisualConsensus:
    """Independent of the drifting predictor; repeated frames never count."""
    def __init__(self):
        self.last = None
        self.count = 0
        self.first_ms = 0

    def observe(self, frame, ms, xy, heading, received, maximum_speed):
        previous = self.last
        if previous and (frame<=previous[0] or ms<=previous[1]):
            return False
        consistent = False
        if previous:
            dt=(ms-previous[1])/1000
            consistent=(0<dt<=.5 and 0<=received-previous[4]<=.5 and
                        math.dist(xy,previous[2])<=maximum_speed*dt and
                        abs(wrap_angle(heading-previous[3]))<=3.*dt+.1)
        self.count = self.count+1 if consistent else 1
        if not consistent:self.first_ms=ms
        self.last=(frame,ms,list(xy),heading,received)
        return self.count>=3 and ms-self.first_ms>=200


class DelayedEstimator:
    def __init__(self, tau=1.5, history_sec=4.0):
        self.tau = tau
        self.history_sec = history_sec
        self.state = None
        self.t = None
        self.history = deque()

    def initialize(self, pose, measurement_time, now, yaw_rate, speed_target):
        self.state = [*pose, 0.0]
        self.t = measurement_time
        self.history.clear()
        self.advance(now, yaw_rate, speed_target)

    def advance(self, now, yaw_rate, speed_target):
        if self.state is None or now <= self.t:
            return
        # Small replay steps preserve a usable IMU history over the visual delay.
        while self.t < now - 1e-9:
            end = min(now, self.t + 0.02)
            self.history.append([self.t, end, list(self.state), yaw_rate, speed_target])
            self.state = propagate(self.state, end - self.t, yaw_rate, speed_target, self.tau)
            self.t = end
        while self.history and self.history[0][1] < now - self.history_sec:
            self.history.popleft()

    def correct(self, pose, stamp, gain, heading_gain, speed=None, max_innovation=3.0):
        if not self.history or stamp < self.history[0][0] or stamp > self.t:
            return dict(accepted=False, reason='outside_history')
        index = next((i for i, s in enumerate(self.history) if s[0] <= stamp <= s[1]), None)
        if index is None:
            return dict(accepted=False, reason='outside_history')
        segments = list(self.history)
        start, end, old, yaw_rate, target = segments[index]
        at = propagate(old, stamp - start, yaw_rate, target, self.tau)
        dx, dy = pose[0] - at[0], pose[1] - at[1]
        dh = wrap_angle(pose[2] - at[2])
        if math.hypot(dx, dy) > max_innovation:
            return dict(accepted=False, reason='position_outlier', innovation_m=math.hypot(dx, dy))
        state = [at[0] + gain * dx, at[1] + gain * dy,
                 wrap_angle(at[2] + heading_gain * dh), at[3]]
        if speed is not None:
            state[3] += gain * (speed - state[3])
        rebuilt = segments[:index]
        if stamp > start:
            rebuilt.append([start, stamp, old, yaw_rate, target])
        for a, b, _, rate, thrust in [[stamp, end, state, yaw_rate, target]] + segments[index + 1:]:
            if b <= a:
                continue
            rebuilt.append([a, b, list(state), rate, thrust])
            state = propagate(state, b - a, rate, thrust, self.tau)
        self.history = deque(rebuilt)
        self.state = state
        return dict(accepted=True, innovation_m=math.hypot(dx, dy),
                    innovation_xy_m=[dx, dy], heading_innovation_rad=dh,
                    historical_prediction=at, corrected_current_state=list(state))
