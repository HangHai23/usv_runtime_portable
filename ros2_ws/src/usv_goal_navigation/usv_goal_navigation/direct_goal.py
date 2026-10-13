"""Direct, gate-corridor-only goal entry. No user routes or obstacle planning."""
import json
import math
import time
from std_msgs.msg import String
from .control import goal_center, route_command


def alignment_correction(pose, gate, c):
    # Positive signed distance is before the gate, regardless of left/right.
    distance = -gate['sign']*(pose[0]-gate['center'][0])
    normal = 0. if gate['sign'] > 0 else math.pi
    error = math.atan2(math.sin(normal-pose[2]), math.cos(normal-pose[2]))
    span = c.get('goal_alignment_distance_m', .2)
    in_opening = gate['y_min'] <= pose[1] <= gate['y_max']
    weight = max(0., min(1., 1-distance/span)) if span > 0 and in_opening and 0 <= distance <= span else 0.
    limit = c.get('goal_alignment_max_yaw_rate_rad_s', .6)
    rate = weight*max(-limit, min(limit, c.get('goal_alignment_kp', .8)*error))
    return dict(goal_alignment_yaw_rate_rad_s=rate, goal_alignment_weight=weight,
                goal_normal_heading_error_rad=error, goal_line_distance_m=distance)


def gate_geometry(ends, side, depth, post_margin):
    if side not in ('left','right') or len(ends)!=2:
        raise ValueError('invalid goal')
    center=list(goal_center(ends))
    if abs(ends[0][0]-ends[1][0])>.3:
        raise ValueError('goal line must align with field Y axis')
    low,high=sorted(p[1] for p in ends)
    if high-low<=2*post_margin:raise ValueError('goal too narrow for configured margin')
    sign=1 if side=='right' else -1
    return dict(center=center,sign=sign,y_min=low+post_margin,y_max=high-post_margin,
                stop_xy_m=[center[0]+sign*depth,center[1]])


def entry_state(pose, visual, age, gate, size, depth):
    x,y=pose[:2]; gx=gate['center'][0];sign=gate['sign']
    entered=sign*(x-gx)
    in_mouth=gate['y_min']<=y<=gate['y_max']
    # Field bounds are display-only. Outside estimates must still be able to
    # navigate toward the gate; entry stopping applies only within its mouth.
    if in_mouth and entered>depth+.5:return 'goal_overshoot_stop'
    if visual is not None and age is not None and age<=1.0:
        measured=sign*(visual[0]-gx)
        if measured>=depth and gate['y_min']<=visual[1]<=gate['y_max']:
            return 'arrived'
    if in_mouth and entered>=depth:return 'goal_entry_verifying'
    return 'navigating'


def update_goal(node):
    now=time.monotonic();c=node.c
    dt=max(0.,min(.2,now-node.last_tick));node.last_tick=now;node.advance(now)
    age=now-node.last_visual if node.last_visual else None
    selected=not c['require_mode'] or (node.auto and now-node.last_mode<=c['mode_timeout'])
    ends=node.map.goals.get(c['goal_side'])
    state='navigating';gate=node.active_gate
    if node.fault:state=node.fault
    elif node.stream_rearm_required:state='stream_recovered_wait_manual'
    elif not selected:state='manual_or_mode_unavailable'
    elif node.est.state is None or ends is None or node.map.size is None:state='waiting_for_map_and_position'
    elif now-node.last_imu>c['imu_timeout']:state='imu_timeout'
    elif age is None or age>c['max_prediction_sec']:state='visual_timeout'
    elif c['armed'] and (node.pwm is None or now-node.last_pwm>c['pwm_timeout']):state='pwm_feedback_timeout'
    elif c['armed'] and (node.pwm.emergency_stop_active or node.pwm.watchdog_active):state='actuator_stopped'
    elif node.arrived:state='arrived'
    if gate is None and ends is not None:
        try:gate=gate_geometry(ends,c['goal_side'],c['goal_entry_depth_m'],c['goal_post_margin_m'])
        except (ValueError,TypeError,IndexError):state='goal_geometry_invalid'
    data={};outputs=[0.,0.];predicted=node.est.state
    if state=='navigating' and gate:
        if node.start is None:
            node.start=predicted[:2];node.active_gate=gate
            node.run_id+=1;node.run_started=now;node.running=True;node.turn_phase.reset()
            node.rec.write('run_start',run_id=node.run_id,start=node.start,map=node.map.snapshot(),gate=gate)
            plan=dict(schema='usv.route_draft.v1',units='m',coord=node.map.coord,
                      waypoints_m=[node.start,gate['stop_xy_m']],goal_side=c['goal_side'],
                      goal_entry_depth_m=c['goal_entry_depth_m'],cruise_speed_m_s=c['cruise_speed_m_s'],
                      maximum_yaw_rate_rad_s=c['maximum_yaw_rate_rad_s'])
            if math.dist(node.start,gate['stop_xy_m'])>=.01:
                (node.rec.directory/f'route_run_{node.run_id:04d}.json').write_text(json.dumps(plan,indent=2))
                if not (node.rec.directory/'route.json').exists():
                    (node.rec.directory/'route.json').write_text(json.dumps(plan,indent=2))
            node.rec.write('goal_plan',run_id=node.run_id,plan=plan)
        state=entry_state(predicted,node.previous_visual,age,gate,node.map.size,c['goal_entry_depth_m'])
        if getattr(node,'goal_stop_latched',False):
            # Already stopped at the entry plane: passive drift must not turn
            # this completed stop into a permanent fault or restart propulsion.
            v=node.previous_visual
            confirmed=(v is not None and age is not None and age<=1.0 and
                       gate['sign']*(v[0]-gate['center'][0])>=c['goal_entry_depth_m'] and
                       gate['y_min']<=v[1]<=gate['y_max'])
            state='arrived' if confirmed else 'goal_entry_verifying'
        if state in ('goal_entry_verifying','arrived'):
            node.goal_stop_latched=True
        if state in ('goal_boundary_stop','goal_overshoot_stop'):
            node.fault=state
        if state=='arrived':node.arrived=True
        if state=='navigating':
            # Aim beyond the stopping plane so no arrival radius/slowdown can
            # halt the vessel in front of the gate. Stop is handled above.
            stop=gate['stop_xy_m'];dx=stop[0]-node.start[0];dy=stop[1]-node.start[1]
            length=max(1e-6,math.hypot(dx,dy))
            target=[stop[0]+dx/length*c['lookahead_m'],stop[1]+dy/length*c['lookahead_m']]
            config=dict(c,goal_runthrough=True)
            alignment=alignment_correction(predicted,gate,c)
            config['goal_alignment_yaw_rate_rad_s']=alignment['goal_alignment_yaw_rate_rad_s']
            if age>c['prediction_slow_after_sec']:
                config['cruise_speed_m_s']=min(c['cruise_speed_m_s'],c['prediction_speed_cap_m_s'])
            left,right,data=route_command(predicted,node.start,target,node.rate,config,node.turn_phase)
            data.update(alignment)
            delta=c['output_slew_rate_percent_s']*dt
            outputs=[p+max(-delta,min(delta,q-p)) for p,q in zip(node.outputs,[left,right])]
            if data.get('single_motor_turn'):
                outputs=[0. if q==0 else p for p,q in zip(outputs,[left,right])]
    node.outputs=outputs
    active=c['armed'] and state=='navigating'
    node.publish(*(outputs if active else [0.,0.]))
    data.update(start_xy_m=node.start,target_xy_m=gate['stop_xy_m'] if gate else None)
    if node.start and gate and predicted:
        a,b=node.start,gate['stop_xy_m'];dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        u=max(0.,min(1.,((predicted[0]-a[0])*dx+(predicted[1]-a[1])*dy)/max(length*length,1e-9)))
        data.update(route_segment=0,route_progress=u,
                    path_error_m=math.hypot(predicted[0]-a[0]-u*dx,predicted[1]-a[1]-u*dy))
    if 'desired_yaw_rate_rad_s' in data:
        data['yaw_rate_error_rad_s']=node.rate-data['desired_yaw_rate_rad_s']
        data['estimated_speed_error_m_s']=predicted[3]-data['desired_speed_m_s']
    status=dict(state=state,run_id=node.run_id,command_active=active,armed=c['armed'],
        goal_side=c['goal_side'],automatic_selected=selected,frame=node.frame,
        navigation_phase='goal_entry',route_xy_m=None,route_coord=None,
        map=node.map.snapshot(),predicted_pose=predicted,estimated_speed_m_s=predicted[3] if predicted else None,
        gate=gate,goal_entry_depth_m=c['goal_entry_depth_m'],
        entry_depth_m=gate['sign']*(predicted[0]-gate['center'][0]) if gate and predicted else None,
        visual_age_sec=age,dead_reckoning=age is not None and age>.25,
        navigation_degraded=age is None or age>c['prediction_slow_after_sec'],
        map_age_sec=now-node.last_map if node.last_map else None,
        yaw_rate_rad_s=node.rate,raw_yaw_rate_rad_s=node.raw_rate,
        effective_pwm_us=node.effective_pwm(now),
        pulse_width_us=[int(v) for v in node.pwm.pulse_width_us] if node.pwm else None,
        applied_percent=[float(v) for v in node.pwm.applied_percent] if node.pwm else None,
        pwm_feedback_age_sec=now-node.last_pwm if node.last_pwm else None,
        pwm_source=node.pwm.source if node.pwm else None,
        actuator_emergency_stop=node.pwm.emergency_stop_active if node.pwm else None,
        dry_run=node.pwm.dry_run if node.pwm else None,
        arrival_basis='accepted_visual_position',
        uncertainty_estimate_m=c['uncertainty_initial_m']+(age or 0)*c['uncertainty_growth_m_s'],
        proposed_left_percent=outputs[0],proposed_right_percent=outputs[1],
        correction=node.last_correction,counts=node.counts.copy(),**data)
    node.status_pub.publish(String(data=json.dumps(status,allow_nan=False)));node.rec.write('tick',**status)
    if state in ('arrived','goal_boundary_stop','goal_overshoot_stop'):node.end_run(state)
    if not selected:
        node.goal_stop_latched=False
        node.end_run('manual_takeover');node.start=None;node.arrived=False;node.active_gate=None;node.turn_phase.reset()
