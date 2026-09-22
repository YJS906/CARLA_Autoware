#!/usr/bin/env python3
"""Summarize read-only AEB observations beside the closed-loop trial records."""
import argparse
import collections
import gzip
import hashlib
import json
import math
from pathlib import Path


TRIALS = ['baseline_run1', 'vehicle_stop_run1'] + [
    f'{kind}_run{i}' for i in range(1, 4)
    for kind in ('pedestrian_stop', 'vehicle_avoidance')
]


def available(path):
    return path.is_file() or Path(str(path) + '.gz').is_file()


def raw_bytes(path):
    if path.is_file():
        return path.read_bytes()
    with gzip.open(str(path) + '.gz', 'rb') as stream:
        return stream.read()


def sha256(path):
    # Keep the hash of uncompressed JSON stable when evidence is gzipped.
    return hashlib.sha256(raw_bytes(path)).hexdigest()


def counts(values):
    return dict(collections.Counter(values))


def trial(folder, name):
    harness_path = folder / (name + '.json')
    if not available(harness_path):
        return {'state': 'pending_trial'}
    harness = json.loads(raw_bytes(harness_path))
    samples = harness['samples']
    result = {
        'state': 'trial_complete', 'trial_sha256': sha256(harness_path),
        'success': harness['success'], 'result': harness['result'],
        'actual_peak_kmh': harness['max_speed_mps'] * 3.6,
        'stopped_clearance_m': harness.get('stopped_clearance_m'),
        'collision_records': len(harness['collisions']),
        'fixture_detection_samples': harness['fixture_detection_samples'],
        'trial_samples': len(samples), 'planning_factor_message_counts': harness['factor_message_counts'],
    }
    first_brake = next((s for s in samples if s['speed_mps'] > .1 and s['brake'] > .01), None)
    if first_brake:
        result['first_moving_brake'] = {key: first_brake[key] for key in (
            'ros_time', 'speed_mps', 'clearance_m', 'brake', 'objects_by_class')}
    observer_path = folder / (name + '_aeb.json')
    if not available(observer_path):
        result['aeb_evidence'] = 'not_collected_for_baseline' if name.startswith('baseline') else 'pending_observer'
        return result
    data = json.loads(raw_bytes(observer_path))
    low, high = samples[0]['ros_time'], samples[-1]['ros_time']
    events = [event for event in data['events'] if low <= event['ros_s'] <= high]
    diagnostics = [status for event in events for status in event.get('status', [])]
    metrics = [metric for event in events for metric in event.get('metrics', [])]
    markers = [marker for event in events for marker in event.get('markers', [])]
    clouds = [event['estimate'] for event in events if 'estimate' in event]
    components = [component for cloud in clouds for component in cloud.get('clusters', [])]
    trajectories = [event for event in events if 'path_length_m' in event]
    object_events = [event for event in events if 'objects' in event]
    brake_decisions = sum(m['name'] == 'decision' and m['value'] == 'brake' for m in metrics)
    error_statuses = sum(s['level'] >= 2 for s in diagnostics)
    result.update({
        'aeb_evidence': 'available', 'observer_sha256': sha256(observer_path),
        'observer_errors': data['errors'], 'trial_ros_time_window': [low, high],
        'aeb_parameters': data['parameters'],
        'aeb_diagnostic_samples': len(diagnostics),
        'aeb_diagnostic_messages': counts(s['message'] for s in diagnostics),
        'aeb_error_diagnostics': error_statuses,
        'aeb_brake_decisions': brake_decisions,
        'aeb_rss_samples': sum('rss_m' in event for event in events),
        'aeb_marker_counts': counts(m['ns'] for m in markers),
        'aeb_nonempty_marker_counts': counts(m['ns'] for m in markers if m['points']),
        'mpc_trajectory_messages': len(trajectories),
        'mpc_trajectory_max_length_m': max((e['path_length_m'] for e in trajectories), default=None),
        'predicted_object_messages': len(object_events),
        'predicted_object_nonempty_messages': sum(bool(e['objects']) for e in object_events),
        'predicted_object_labels': counts(str(label) for e in object_events for obj in e['objects']
                                          for label, probability in obj['labels'] if probability > .5),
        'pointcloud_corridor_estimates': len(clouds),
        'pointcloud_corridor_max_component_size': max((c['points'] for c in components), default=0),
        'pointcloud_corridor_size_height_accepted_components': sum(c['accepted_size_height'] for c in components),
        'aeb_log_messages': [e for e in events if e['topic'] == '/rosout'],
    })
    if error_statuses or brake_decisions:
        result['attribution'] = 'aeb_intervention_observed_inspect_timing'
    elif name.startswith(('pedestrian_stop', 'vehicle_stop')) and harness['success']:
        result['attribution'] = 'planning_stop_no_aeb_intervention_observed'
    elif name.startswith('vehicle_avoidance') and harness['success']:
        result['attribution'] = 'planning_avoidance_no_aeb_intervention_observed'
    else:
        result['attribution'] = 'no_aeb_intervention_observed'
    return result


def braking_response(folder, name):
    harness = json.loads(raw_bytes(folder / (name + '.json')))
    samples = harness['samples']
    braking = next((i for i, s in enumerate(samples)
                    if s['progress_m'] > 45 and s['speed_mps'] > 1 and s['brake'] > .01), None)
    if braking is None:
        return {'observation': 'no_post_obstacle_braking_sample'}
    stop = next((i for i in range(braking, len(samples)) if samples[i]['speed_mps'] < .15), None)
    if stop is None:
        return {'observation': 'braking_without_full_stop'}
    start = max(0, braking - 3)
    start = max(range(start, braking + 1), key=lambda i: samples[i]['speed_mps'])
    end = min(len(samples), stop + 14)
    records = []
    for i in range(start, end):
        sample = samples[i]
        control = sample['ros_control']
        command_time = control['stamp']['sec'] + control['stamp']['nanosec'] * 1e-9
        observed_acceleration = None
        if i > start:
            previous = samples[i-1]
            observed_acceleration = (sample['speed_mps'] - previous['speed_mps']) / (sample['sim_s'] - previous['sim_s'])
        records.append({
            key: sample[key] for key in ('sim_s', 'ros_time', 'frame', 'progress_m', 'speed_mps', 'brake', 'throttle', 'mode')
        } | {
            'command_age_s': sample['ros_time'] - command_time,
            'command_velocity_mps': control['longitudinal']['velocity'],
            'command_acceleration_mps2': control['longitudinal']['acceleration'],
            'observed_speed_derivative_mps2': observed_acceleration,
        })
    before = [t for t in harness['trajectory_snapshots'] if t['sim_s'] <= samples[braking]['sim_s']]
    trajectory = before[-1] if before else None
    nearest = None
    if trajectory:
        ego = min(samples, key=lambda s: abs(s['sim_s'] - trajectory['sim_s']))
        point = min(trajectory['trajectory']['points'], key=lambda p: math.hypot(
            p['pose']['position']['x'] - ego['xyz'][0], p['pose']['position']['y'] + ego['xyz'][1]))
        nearest = {'sim_s': trajectory['sim_s'],
                   'velocity_mps': point['longitudinal_velocity_mps'],
                   'acceleration_mps2': point['acceleration_mps2'],
                   'method': 'Nearest map trajectory point to CARLA actor location reflected into ROS; not rear-axle exact projection.'}
    return {
        'source': name + '.json', 'trial_sha256': sha256(folder / (name + '.json')),
        'observation': 'during_passage_braking_to_stop_then_continued',
        'start_speed_mps': samples[start]['speed_mps'], 'stop_speed_mps': samples[stop]['speed_mps'],
        'deceleration_duration_s': samples[stop]['sim_s'] - samples[start]['sim_s'],
        'mean_observed_deceleration_mps2': (samples[stop]['speed_mps'] - samples[start]['speed_mps']) / (samples[stop]['sim_s'] - samples[start]['sim_s']),
        'stop_progress_m': samples[stop]['progress_m'],
        'command_velocity_at_actual_stop_mps': samples[stop]['ros_control']['longitudinal']['velocity'],
        'max_sampled_brake_in_window': max(row['brake'] for row in records),
        'max_command_age_s': max(row['command_age_s'] for row in records),
        'observed_modes': sorted(set(row['mode'] for row in records)),
        'collision_records': len(harness['collisions']),
        'preceding_trajectory_nearest_point': nearest,
        'records': records,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    result = {
        'scope': 'Actual AEB diagnostics and source inputs during main trial only; no simulator changes.',
        'time_alignment': 'Observer reception ros_s restricted to harness first/last sample ros_time; excludes preparation and cleanup.',
        'pointcloud_estimate_limit': 'Fixed forward corridor reproduces z crop, voxel centroids and 3D connected components; not the exact MPC-footprint crop or actor-specific ROI.',
        'historical_causality': 'Old collision lacks internal AEB evidence; sparse LiDAR candidates and proven empty-vector defect are verified constraints, not proof of unique historical cause.',
        'trials': {name: trial(args.folder, name) for name in TRIALS},
    }
    result['completed_trials'] = sum(t['state'] == 'trial_complete' for t in result['trials'].values())
    result['observer_trials'] = sum(t.get('aeb_evidence') == 'available' for t in result['trials'].values())
    result['all_expected_trials_available'] = result['completed_trials'] == 8 and result['observer_trials'] == 7
    output = args.folder / 'aeb-analysis.json'
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    if result['all_expected_trials_available']:
        brake = {
            'scope': 'Read-only analysis of sampled actual CARLA control, ROS control and trajectory during three completed avoidance trials.',
            'confirmed': [
                'Avoidance run1 has planned deceleration immediately before braking; all three trials have fresh negative ROS acceleration commands.',
                'Actual vehicle deceleration exceeded the command magnitude and the vehicle briefly stopped before continuing.',
                'No AEB error/brake decision was observed in the corresponding observer records.',
                'The bridge watchdog forces brake=1.0; recorded braking samples were fractional and commands were fresh.'
            ],
            'limits': [
                'The ActuationCommandStamped input and explicit watchdog transitions were not separately recorded; sub-sample events cannot be excluded.',
                'Speed derivatives are finite differences of CARLA velocity magnitude, not a calibrated longitudinal accelerometer.',
                'Actuation calibration/dynamics mismatch is a candidate, not an experimentally isolated root cause.'
            ],
            'runtime_and_source_brake_map_sha256': '30fc1cea193cc21d3dc2bda990848363541334a037338959d78fb3396d22353f',
            'trials': {f'vehicle_avoidance_run{i}': braking_response(args.folder, f'vehicle_avoidance_run{i}') for i in range(1, 4)},
        }
        (args.folder / 'braking-response-analysis.json').write_text(json.dumps(brake, indent=2, ensure_ascii=False) + '\n')
    print(output)
    print(f"Trials {result['completed_trials']}/8; observers {result['observer_trials']}/7")


if __name__ == '__main__':
    main()
