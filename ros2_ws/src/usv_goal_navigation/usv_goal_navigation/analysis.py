"""Offline fitting. Results are candidates, never automatically applied."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import yaml


def fit_gain(t, u, y, tau):
    if len(t) < 30 or np.std(u) < 5:
        return dict(usable=False, reason='insufficient_excitation')
    # Fit the one-step exact first-order response, not noisy derivatives.
    dt = np.diff(t)
    mask = (dt > 0.005) & (dt < 0.5)
    a = np.exp(-dt[mask]/tau)
    x = (1-a)*u[:-1][mask]
    target = y[1:][mask] - a*y[:-1][mask]
    A = np.column_stack([x, 1-a])
    if len(x) < 20 or np.linalg.matrix_rank(A) < 2:
        return dict(usable=False, reason='insufficient_samples')
    k, bias = np.linalg.lstsq(A, target, rcond=None)[0]
    residual = target-A @ np.array([k, bias])
    return dict(usable=bool(k > 0), gain=float(k), offset=float(bias),
                rmse=float(np.sqrt(np.mean(residual**2))), samples=len(x), tau_sec=tau)


def estimate_delay(imu_t, rate, visual_t, heading):
    """Align observed heading increments to time-integrated IMU, using receipt time."""
    if len(imu_t) < 100 or len(visual_t) < 15 or np.std(rate) < 0.025:
        return dict(usable=False, reason='need_multiple_turns_and_variable_yaw_rate')
    visual_heading = np.unwrap(heading)
    # Adjacent-frame differences amplify timestamp jitter and AprilTag angle noise.
    # Compare fixed 0.6s windows and exclude spans crossing missing observations.
    a = np.arange(max(visual_t[0], imu_t[0]+2.0),
                  min(visual_t[-1], imu_t[-1])-.6, .05)
    b = a+.6
    gaps = np.r_[0, np.cumsum(np.diff(visual_t)>.5)]
    left = np.maximum(0, np.searchsorted(visual_t,a,side='right')-1)
    right = np.minimum(len(visual_t)-1,np.searchsorted(visual_t,b))
    good = gaps[right] == gaps[left]
    a, b = a[good], b[good]
    delta = np.interp(b,visual_t,visual_heading)-np.interp(a,visual_t,visual_heading)
    integrated = np.r_[0., np.cumsum(np.diff(imu_t)*(rate[1:]+rate[:-1])*0.5)]
    scores = []
    for delay in np.arange(0., 2.001, 0.02):
        valid = (a-delay >= imu_t[0]) & (b-delay <= imu_t[-1])
        if np.sum(valid) < 12:
            continue
        predicted = np.interp(b[valid]-delay, imu_t, integrated)-np.interp(a[valid]-delay, imu_t, integrated)
        error = delta[valid]-predicted
        scores.append([float(delay), float(np.sqrt(np.mean(error**2)))])
    if not scores:
        return dict(usable=False, reason='insufficient_overlap')
    best = min(scores, key=lambda p: p[1])
    worst = max(p[1] for p in scores)
    usable = best[0] not in (0., 2.) and worst > best[1]*1.2 and best[1] < 0.15
    return dict(usable=bool(usable), estimated_total_delay_sec=best[0], heading_increment_rmse_rad=best[1],
                scan=scores, note='Includes camera/processing/transport lag; clock-independent motion alignment, not pure network latency.')


def analyze(directory):
    directory = Path(directory)
    parameters = json.loads((directory/'parameters.json').read_text())
    rows = []
    damaged = 0
    with (directory/'telemetry.jsonl').open() as stream:
        for line in stream:
            try:
                rows.append(json.loads(line))
            except ValueError:
                damaged += 1
    imu = [r for r in rows if r['event'] == 'imu']
    visual = [r for r in rows if r['event'] == 'visual' and r['correction']['accepted']]
    ticks = [r for r in rows if r['event'] == 'tick']
    runs = [r for r in rows if r['event'] in ('run_start', 'run_end')]
    sign = -1 if parameters['imu_yaw_rate_reversed'] else 1
    it = np.array([r['monotonic'] for r in imu])
    ir = np.array([(r['message']['angular_velocity']['z']-parameters['imu_bias_rad_s'])*sign for r in imu])
    vt = np.array([r['received_monotonic'] for r in visual])
    vh = np.array([r['heading'] for r in visual])
    delay = estimate_delay(it, ir, vt, vh)
    opposite = estimate_delay(it, -ir, vt, vh)
    invert_recorded_rate = (opposite.get('usable',False) and
        opposite.get('heading_increment_rmse_rad',float('inf')) <
        .7*delay.get('heading_increment_rmse_rad',float('inf')))
    if invert_recorded_rate:
        delay = opposite
        ir = -ir
    lag = delay.get('estimated_total_delay_sec', parameters['measurement_delay_sec']) if delay.get('usable') else parameters['measurement_delay_sec']
    good_ticks = [r for r in ticks if r.get('pwm_feedback_age_sec') is not None
                  and r['pwm_feedback_age_sec'] < parameters['pwm_timeout'] and not r.get('dry_run')]
    tt = np.array([r['monotonic'] for r in good_ticks])
    mean_pwm = np.array([sum(r['effective_pwm_us'])/2 for r in good_ticks])
    diff_pwm = np.array([(r['effective_pwm_us'][1]-r['effective_pwm_us'][0])*parameters['motor_turn_sign'] for r in good_ticks])
    yaw = np.array([r['yaw_rate_rad_s'] for r in good_ticks]) * (-1 if invert_recorded_rate else 1)
    turn = fit_gain(tt, diff_pwm, yaw, parameters['yaw_time_constant_sec'])
    speed_fit = dict(usable=False, reason='insufficient_visual_motion')
    if len(vt) >= 15 and len(tt) >= 30:
        positions = np.array([r['xy_m'] for r in visual])
        intervals = np.diff(np.array([r['measurement_ms'] for r in visual]))/1000
        valid = (intervals > 0.04) & (intervals < 0.8)
        mid = (vt[1:]+vt[:-1])/2 - lag
        valid &= (mid >= tt[0]) & (mid <= tt[-1])
        heading = np.unwrap(vh)
        heading_mid = (heading[1:]+heading[:-1])/2
        velocity = np.diff(positions, axis=0)/np.maximum(intervals[:, None], 1e-6)
        speed = velocity[:, 0]*np.cos(heading_mid)+velocity[:, 1]*np.sin(heading_mid)
        pwm = np.interp(mid[valid], tt, mean_pwm)
        speed_fit = fit_gain(mid[valid], pwm, speed[valid], parameters['speed_time_constant_sec'])
    track = [r['cross_track_m'] for r in ticks if 'cross_track_m' in r]
    innovations = [r['correction'].get('innovation_m') for r in visual if 'innovation_m' in r['correction']]
    sequences = {}
    packet_stats = dict(received=0, malformed=0, apparent_missing=0, duplicate_or_reordered=0)
    for row in rows:
        if row['event'] != 'udp':
            continue
        packet_stats['received'] += 1
        try:
            raw = json.loads(row['raw'])
            packet = json.loads(bytes.fromhex(raw['payload_hex']))
            session, seq = packet['session'], packet['seq']
            if type(seq) is not int or not isinstance(session, str):
                raise ValueError('invalid sequence')
            previous = sequences.get(session)
            if previous is not None:
                if seq <= previous:
                    packet_stats['duplicate_or_reordered'] += 1
                else:
                    packet_stats['apparent_missing'] += max(0, seq-previous-1)
            sequences[session] = max(seq, previous if previous is not None else seq)
        except (ValueError, KeyError, TypeError):
            packet_stats['malformed'] += 1
    report = dict(samples=dict(imu=len(imu), visual=len(visual), ticks=len(ticks), damaged_lines=damaged),
        runs=runs, packets=packet_stats, delay=delay, invert_recorded_yaw_for_alignment=bool(invert_recorded_rate), turn_gain_fit=turn, speed_gain_fit=speed_fit,
        cross_track_rms_m=float(np.sqrt(np.mean(np.square(track)))) if track else None,
        mean_position_innovation_m=float(np.mean(innovations)) if innovations else None,
        limitations=['All gains are candidates under a first-order model.',
                    'No independent absolute-position truth: mean innovation is not a calibrated position offset.',
                    'Separate tag-to-hull offset, water drift and latency with multiple left/right turns and speed changes.',
                    'PWM telemetry is commanded output, not measured motor thrust or oscilloscope feedback.'])
    (directory/'analysis.json').write_text(json.dumps(report, indent=2))
    suggestions = {}
    if invert_recorded_rate:
        suggestions['imu_yaw_rate_reversed'] = not parameters['imu_yaw_rate_reversed']
    if delay.get('usable'):
        suggestions['measurement_delay_sec'] = delay['estimated_total_delay_sec']
    if turn.get('usable'):
        suggestions['turn_gain_rad_s_per_us'] = turn['gain']
    if speed_fit.get('usable'):
        suggestions['speed_gain_m_s_per_us'] = speed_fit['gain']
    (directory/'candidate_parameters.yaml').write_text('# 候选值，须结合路线与残差审核后手动应用。\n' +
        yaml.safe_dump({'goal_navigation': {'ros__parameters': suggestions}}, sort_keys=False))
    columns = ['monotonic', 'unix_ns', 'run_id', 'state', 'visual_age_sec', 'predicted_pose',
               'yaw_rate_rad_s', 'pulse_width_us', 'effective_pwm_us', 'distance_m',
               'cross_track_m', 'desired_speed_m_s', 'desired_yaw_rate_rad_s',
               'proposed_left_percent', 'proposed_right_percent']
    with (directory/'navigation.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(ticks)
    with (directory/'visual.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['monotonic', 'received_monotonic',
            'measurement_ms', 'frame', 'xy_m', 'heading', 'speed_m_s', 'correction'], extrasaction='ignore')
        writer.writeheader()
        writer.writerows(visual)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', help='session directory containing telemetry.jsonl')
    args = parser.parse_args()
    report = analyze(args.directory)
    print(json.dumps({k:v for k,v in report.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
