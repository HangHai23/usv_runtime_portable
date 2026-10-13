"""Read-only field view. Cached geometry is separate from fresh observations."""
import copy
import json
import math
import time


def point(value):
    return (isinstance(value, (list, tuple)) and len(value) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


class GlobalView:
    def __init__(self):
        self.identity = None
        self.geometry = {'coord': None, 'size_m': None, 'goals': {}}
        self.objects = {}; self.frame = None; self.received = None
        self.navigation = {}; self.nav_received = None; self.trace = []
        self.mission = {}; self.mission_received = None
        self.receiver = {}; self.packet_received = None

    def mission_status(self, raw):
        try:
            report = json.loads(raw)
            if not isinstance(report, dict) or not isinstance(report.get('stage'), str): return
            json.dumps(report, allow_nan=False)
            self.mission = report
            self.mission_received = time.monotonic()
        except (ValueError, TypeError): return

    def observation(self, raw):
        now = time.monotonic()
        try:
            r = json.loads(raw)
            json.dumps(r, allow_nan=False)
            o = r.get('observation')
            self.receiver = dict(counts=r.get('receiver_counts') or {},
                own_boat_valid=bool(r.get('valid')),
                field_status=o.get('field_status') if isinstance(o,dict) else None,
                boat_count=len(o.get('boats') or []) if isinstance(o,dict) else 0)
            self.packet_received = r.get('received_monotonic')
            if r.get('reset_required'):
                self.__init__(); return
            if not isinstance(o, dict) or not isinstance(o.get('coord'), str): return
            identity = (r.get('session'), o['coord'])
            if identity != self.identity:
                receiver, packet_received = self.receiver, self.packet_received
                self.__init__(); self.identity = identity; self.geometry['coord'] = o['coord']
                self.receiver, self.packet_received = receiver, packet_received
            # 'valid' describes own-boat localization, not the field geometry.
            if o.get('field_valid'):
                size = o.get('size_m')
                if point(size) and min(size) > 0 and self.geometry['size_m'] is None:
                    self.geometry['size_m'] = size
                for g in o.get('goals', []):
                    ends = g.get('ends_m')
                    if g.get('side') in ('left', 'right') and isinstance(ends, list) and len(ends) == 2 and all(point(p) for p in ends):
                        self.geometry['goals'].setdefault(g['side'], ends)
            frame = o.get('frame')
            if type(frame) is not int or (self.frame is not None and frame <= self.frame): return
            received = r.get('received_monotonic', now)
            if type(received) not in (int, float) or not math.isfinite(received) or not 0 <= now-received < 10: return
            own = r.get('own_boat') or {}; heading = own.get('heading_rad')
            vessel = None
            if r.get('valid') and point(own.get('xy_m')) and type(heading) in (int, float) and math.isfinite(heading):
                vessel = dict(xy_m=own['xy_m'], heading_rad=heading, id=own.get('id'))
            ball = o.get('ball') or {}
            ball = dict(xy_m=ball['xy_m'], score=ball.get('score')) if ball.get('state') == 'detected' and point(ball.get('xy_m')) else None
            boats = [dict(id=b.get('id'), xy_m=b['xy_m']) for b in o.get('boats', []) if point(b.get('xy_m'))][:32]
            self.objects = dict(vessel=vessel, ball=ball, boats=boats)
            self.frame = frame; self.received = received
        except (ValueError, TypeError, AttributeError, KeyError):
            return

    def status(self, raw):
        try:
            r = json.loads(raw)
            if not isinstance(r, dict): return
            if not isinstance(r.get('map'), dict): return
            json.dumps(r, allow_nan=False)
            self.navigation = {k: r.get(k) for k in (
                'state', 'run_id', 'command_active', 'goal_side', 'automatic_selected',
                'gate', 'goal_entry_depth_m', 'entry_depth_m', 'yaw_rate_rad_s',
                'navigation_phase', 'navigation_degraded', 'map_age_sec', 'approach_xy_m',
                'obstacle_plan', 'obstacle_plan_id',
                'visual_age_sec', 'dead_reckoning', 'predicted_pose', 'map',
                'start_xy_m', 'target_xy_m', 'uncertainty_estimate_m', 'predicted_outside_boundary',
                'route_xy_m', 'route_coord', 'route_segment', 'route_progress', 'path_error_m',
                'desired_speed_m_s', 'estimated_speed_m_s', 'reference_xy_m')}
            self.nav_received = time.monotonic()
        except (ValueError, TypeError): return

    def snapshot(self):
        now = time.monotonic()
        nav = copy.deepcopy(self.navigation)
        nav_map = nav.get('map') or {}
        if self.geometry['coord'] and nav_map.get('coord') != self.geometry['coord']:
            nav = {}
        geometry = copy.deepcopy(self.geometry)
        if nav and nav_map.get('size_m'):
            geometry = copy.deepcopy(nav_map)
        return dict(geometry=geometry, objects=copy.deepcopy(self.objects),
                    receiver=copy.deepcopy(self.receiver),
                    packet_age_sec=now-self.packet_received if type(self.packet_received) in (int,float) else None,
                    frame=self.frame, session=self.identity[0] if self.identity else None,
                    observation_age_sec=now-self.received if self.received is not None else None,
                    navigation_age_sec=now-self.nav_received if self.nav_received is not None else None,
                    navigation=nav, trace=list(self.trace), mission=copy.deepcopy(self.mission),
                    mission_age_sec=now-self.mission_received if self.mission_received is not None else None)
