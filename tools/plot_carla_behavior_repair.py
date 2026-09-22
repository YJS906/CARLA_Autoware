#!/usr/bin/env python3
"""Analyze/plot repeated CARLA trials without simulator access or raw-file edits.

Examples:
  python3 tools/plot_carla_behavior_repair.py
  python3 tools/plot_carla_behavior_repair.py --input-dir /tmp/trials --before-dir docs/validation/carla-closed-loop-20260922

Pass labels require the recorded harness success AND independently observable
physical criteria. Missing/incomplete runs are not silently counted as passed.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

STOP_MPS = .15
CASES = ('baseline', 'vehicle_stop', 'vehicle_avoidance', 'pedestrian_stop')
CASE_NAMES = {'baseline':'Baseline', 'vehicle_stop':'Roadblock stop',
              'vehicle_avoidance':'Parked-car avoidance', 'pedestrian_stop':'Pedestrian stop'}
COLORS = ('#1768ac', '#16806a', '#8546a8', '#b26c00', '#bd425c')


def stop_runs(samples, minimum_motion_mps=2.0):
    runs, active, peak = [], [], 0.0
    for row in samples:
        peak = max(peak, row['speed_mps'])
        qualifies = row['speed_mps'] < STOP_MPS and peak >= minimum_motion_mps and row['progress_m'] > 5
        if qualifies:
            active.append(row)
        elif active:
            runs.append(active)
            active = []
    if active:
        runs.append(active)
    return [{'start_s': run[0]['sim_s'], 'end_s': run[-1]['sim_s'],
             'duration_s':run[-1]['sim_s']-run[0]['sim_s'],
             'start_frame':run[0]['frame'], 'end_frame':run[-1]['frame'],
             'start_clearance_m':run[0].get('clearance_m'),
             'end_clearance_m':run[-1].get('clearance_m'),
             'min_clearance_m':min((r['clearance_m'] for r in run if r.get('clearance_m') is not None),default=None)}
            for run in runs]


def brief_sample(row):
    return {key:row.get(key) for key in ('sim_s','frame','progress_m','clearance_m','brake')} | {
        'speed_kmh':row['speed_mps']*3.6}


def analyse_one(path, report):
    samples = report.get('samples', [])
    collisions = report.get('collisions', [])
    frame_step = report.get('fixed_delta_seconds') or .05
    timing_consistent = all(math.isclose(b['sim_s']-a['sim_s'],
        (b['frame']-a['frame'])*frame_step, abs_tol=1e-5) for a,b in zip(samples,samples[1:]))
    gaps = [s['clearance_m'] for s in samples if s.get('clearance_m') is not None]
    minimum_motion_mps = report.get('minimum_motion_mps', 2.0)
    required_stop_hold_s = report.get('required_stop_hold_s', 2.0)
    stops = stop_runs(samples, minimum_motion_mps)
    matched = [s for s in samples if any(m.get('closest_prediction_m') is not None and
        m['closest_prediction_m'] < 3 for m in s.get('matches', []))]
    brakes = [s for s in samples if s.get('brake',0) > .01 and s['speed_mps'] > .5]
    latencies = [s['ros_time']-s['object_stamp'] for s in samples
                if s.get('ros_time') is not None and s.get('object_stamp') is not None]
    peak = max((s['speed_mps'] for s in samples),default=0)*3.6
    lateral = max((s.get('lateral_from_original_lane_m',0) for s in samples),default=0)
    progress = max((s['progress_m'] for s in samples),default=0)
    hazard_progress = None
    if report.get('hazards') and report.get('test_start'):
        hazard_progress = report['test_start']['xyz'][1]-report['hazards'][0]['xyz'][1]
    approach = [s for s in samples if hazard_progress is not None and s['progress_m'] <= hazard_progress]
    min_gap = min(gaps,default=None)
    max_stop = max((s['duration_s'] for s in stops),default=0)
    min_collision_frame = min((c['frame'] for c in collisions),default=None)
    preimpact = [s for s in samples if min_collision_frame is not None and s['frame'] < min_collision_frame]
    cleanup = report.get('cleanup',{})
    cleanup_ok = (cleanup.get('stop_verified') is True and not cleanup.get('errors')
                  and not cleanup.get('fatal_error') and not cleanup.get('simulator_physics_frozen'))
    case = report.get('case')
    physical = bool(samples) and not collisions and not report.get('drivable_violations')
    if case == 'baseline':
        physical &= progress >= 30 and peak >= 7.2
    elif case == 'vehicle_avoidance':
        physical &= lateral > 1.5 and min_gap is not None and min_gap > .1 and \
                    report.get('continued_after_pass_s',0) >= 3-1e-6
    elif case in ('vehicle_stop','pedestrian_stop'):
        physical &= max_stop >= required_stop_hold_s-1e-6 and min_gap is not None and min_gap > .1
    else:
        physical = False
    notes = []
    if report.get('success') and not physical:
        notes.append('Harness says success, but independently observed physical criteria disagree.')
    if not timing_consistent:
        notes.append('Recorded sample/frame times disagree; inspect timing before using derived intervals.')
    if peak > report.get('speed_cap_kmh',math.inf)+.5:
        notes.append('Actual peak exceeds configured speed cap by more than 0.5 km/h.')
    if not samples:
        notes.append('No driving samples: setup/error result, not a completed trial.')
    pedestrian_evidence = []
    for actor, state in report.get('pedestrian_states',{}).items():
        stop_frame = state.get('stop_frame')
        hold_time = samples[0]['sim_s'] + (stop_frame-samples[0]['frame'])*frame_step \
            if samples and stop_frame is not None else None
        continuous_after_hold = max((max(0, run['end_s']-max(run['start_s'],hold_time))
                                    for run in stops),default=0) if hold_time is not None else 0
        pedestrian_evidence.append({'actor_id':actor, **state,
            'longest_continuous_ego_stop_after_pedestrian_hold_s':continuous_after_hold,
            'trigger_to_first_collision_s':
                (min_collision_frame-state['trigger_frame'])*frame_step
                if min_collision_frame is not None and state.get('trigger_frame') is not None else None})
    return {'file':path.name, 'case':case, 'run':path.stem, 'raw_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'reported_success':bool(report.get('success')), 'observed_criteria_satisfied':bool(physical),
        'verdict':'PASS' if report.get('success') and physical and cleanup_ok else 'FAIL',
        'result':report.get('result'), 'error':report.get('error'), 'input_mode':report.get('requested_input_mode','not_recorded'),
        'speed_cap_kmh':report.get('speed_cap_kmh'), 'observed_peak_kmh':peak,
        'observed_pre_hazard_peak_kmh':max((s['speed_mps'] for s in approach),default=0)*3.6 if approach else None,
        'hazard_forward_progress_m':hazard_progress,
        'minimum_motion_mps':minimum_motion_mps,
        'required_stop_hold_s':required_stop_hold_s if case in ('vehicle_stop','pedestrian_stop') else None,
        'min_clearance_m':min_gap, 'final_clearance_m':samples[-1].get('clearance_m') if samples else None,
        'max_lateral_m':lateral, 'max_progress_m':progress,
        'final_lateral_m':samples[-1].get('lateral_from_original_lane_m') if samples else None,
        'final_native_lane_id':samples[-1].get('native_lane_id') if samples else None,
        'sample_duration_s':samples[-1]['sim_s']-samples[0]['sim_s'] if samples else 0,
        'sample_count':len(samples), 'collision_callback_count':len(collisions),
        'first_collision_frame':min_collision_frame,
        'last_pre_collision_sample':brief_sample(preimpact[-1]) if preimpact else None,
        'stop_intervals_before_cleanup':stops, 'longest_stop_before_cleanup_s':max_stop,
        'first_moving_brake':brief_sample(brakes[0]) if brakes else None,
        'first_moving_brake_definition':'brake > 0.01 and speed > 0.5 m/s',
        'first_fixture_prediction_match':brief_sample(matched[0]) if matched else None,
        'fixture_prediction_match_samples':len(matched),
        'prediction_class_sample_counts':dict(sum((Counter(s.get('objects_by_class',{})) for s in samples),Counter())),
        'object_age_ros_s_range':[min(latencies),max(latencies)] if latencies else None,
        'factor_message_counts':report.get('factor_message_counts',{}),
        'avoidance_reasons':report.get('avoidance_reasons',{}),
        'rtc_observed_states':report.get('rtc_observed_states',{}),
        'continued_after_pass_s':report.get('continued_after_pass_s'),
        'drivable_violation_count':len(report.get('drivable_violations',[])),
        'native_lane_query_seam_count':len(report.get('native_lane_query_seams',[])),
        'pedestrian_evidence':pedestrian_evidence,
        'cleanup':cleanup, 'cleanup_ok':cleanup_ok, 'frame_step_s':frame_step,
        'sample_frame_time_consistent':timing_consistent, 'notes':notes}


def plot(reports, metrics, before, output):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,
                         'axes.spines.top':False,'axes.spines.right':False})
    fig, axs = plt.subplots(2,3,figsize=(15,9),gridspec_kw={'height_ratios':[1,1.1]})
    fig.subplots_adjust(left=.065,right=.98,bottom=.12,top=.84,wspace=.3,hspace=.38)
    passed = sum(m['verdict']=='PASS' for m in metrics.values())
    fig.suptitle('CARLA Town05 | Autoware behavior repair trials',x=.065,y=.965,ha='left',fontsize=20,fontweight='bold')
    modes = ', '.join(sorted({m['input_mode'] for m in metrics.values()})) or 'no completed inputs'
    fig.text(.065,.92,f'{passed}/{len(metrics)} recorded trials pass observed criteria | Object input: {modes}',fontsize=11,color='#334155')
    fig.text(.065,.884,'Actual measured speeds and body-outline clearances. Repeated runs are shown separately.',fontsize=10,color='#475569')
    for col, case in enumerate(('vehicle_avoidance','pedestrian_stop')):
        ax=axs[0,col]
        ax.set_title(CASE_NAMES[case]+': speed',loc='left',fontweight='bold')
        group=[(name,r) for name,r in reports.items() if r.get('case')==case]
        for i,(name,r) in enumerate(group):
            s=r.get('samples',[]); m=metrics[name]
            label=f"{name.rsplit('_',1)[-1]}: {m['verdict']} ({m['observed_peak_kmh']:.1f} km/h)"
            ax.plot([x['sim_s'] for x in s],[x['speed_mps']*3.6 for x in s],color=COLORS[i%len(COLORS)],label=label,lw=1.8)
        if case in before:
            s=before[case].get('samples',[])
            ax.plot([x['sim_s'] for x in s],[x['speed_mps']*3.6 for x in s],color='#7c8794',ls='--',label='Before repair',lw=1.2)
        caps=sorted({r['speed_cap_kmh'] for _,r in group})
        for cap in caps:ax.axhline(cap,color='#64748b',lw=.8,ls=':',label=f'{cap:g} km/h cap')
        ax.set(xlabel='Simulation time (s)',ylabel='Ego speed (km/h)')
        if group:ax.legend(fontsize=8,loc='best')
    ax=axs[0,2]
    ax.set_title('Baseline / roadblock regression',loc='left',fontweight='bold')
    for i,(name,r) in enumerate((item for item in reports.items() if item[1].get('case') in ('baseline','vehicle_stop'))):
        s=r.get('samples',[]);m=metrics[name]
        ax.plot([x['sim_s'] for x in s],[x['speed_mps']*3.6 for x in s],label=f"{CASE_NAMES[r['case']]}: {m['verdict']}",color=COLORS[i%len(COLORS)],lw=1.8)
    ax.set(xlabel='Simulation time (s)',ylabel='Ego speed (km/h)')
    if ax.lines:ax.legend(fontsize=8)
    ax=axs[1,0]
    ax.set_title('Avoidance: actual CARLA XY path',loc='left',fontweight='bold')
    hazard_marked=False
    for i,(name,r) in enumerate((item for item in reports.items() if item[1].get('case')=='vehicle_avoidance')):
        s=r.get('samples',[])
        if not s:continue
        start=r.get('test_start',{}).get('xyz',s[0]['xyz'])
        ax.plot([p['xyz'][0]-start[0] for p in s],[p['xyz'][1]-start[1] for p in s],label=name.rsplit('_',1)[-1],color=COLORS[i%len(COLORS)],lw=1.8)
        if r.get('hazards') and not hazard_marked:
            point=r['hazards'][0]['xyz']
            ax.scatter([point[0]-start[0]],[point[1]-start[1]],marker='X',s=75,color='#c44048',label='Parked car center',zorder=5)
            hazard_marked=True
    ax.set(xlabel='CARLA X - start X (m)',ylabel='CARLA Y - start Y (m)')
    if ax.lines:ax.legend(fontsize=8)
    ax=axs[1,1]
    ax.set_title('Pedestrian: body-outline clearance',loc='left',fontweight='bold')
    for i,(name,r) in enumerate((item for item in reports.items() if item[1].get('case')=='pedestrian_stop')):
        s=[p for p in r.get('samples',[]) if p.get('clearance_m') is not None]
        ax.plot([p['sim_s'] for p in s],[p['clearance_m'] for p in s],label=f"{name.rsplit('_',1)[-1]}: {metrics[name]['verdict']}",color=COLORS[i%len(COLORS)],lw=1.8)
        for stop in metrics[name]['stop_intervals_before_cleanup']:
            if stop['duration_s']>=metrics[name]['required_stop_hold_s']-1e-6:ax.axvspan(stop['start_s'],stop['end_s'],color=COLORS[i%len(COLORS)],alpha=.08)
    if 'pedestrian_stop' in before:
        s=[p for p in before['pedestrian_stop'].get('samples',[]) if p.get('clearance_m') is not None]
        ax.plot([p['sim_s'] for p in s],[p['clearance_m'] for p in s],color='#7c8794',ls='--',label='Before repair',lw=1.2)
    ax.axhline(0,color='#c44048',lw=.8)
    ax.set(xlabel='Simulation time (s)',ylabel='Clearance (m)')
    if ax.lines:ax.legend(fontsize=8)
    ax=axs[1,2];ax.axis('off')
    ax.set_title('Observed results',loc='left',fontweight='bold')
    rows=[]
    for m in metrics.values():
        gap='—' if m['min_clearance_m'] is None else f"{m['min_clearance_m']:.2f}"
        rows.append([m['run'].replace('vehicle_','car_').replace('pedestrian_','ped_'),m['verdict'],gap,str(m['collision_callback_count'])])
    if rows:
        table=ax.table(cellText=rows,colLabels=['Trial','Result','Gap (m)','Hits'],cellLoc='left',colLoc='left',bbox=[0,.15,1,.78],colWidths=[.5,.17,.19,.14])
        table.auto_set_font_size(False);table.set_fontsize(8)
        for (row,col),cell in table.get_celld().items():
            cell.set_edgecolor('#e2e8f0')
            if row==0:cell.set_facecolor('#edf2f7');cell.set_text_props(fontweight='bold')
            elif col==1:cell.set_text_props(color='#16806a' if rows[row-1][1]=='PASS' else '#bb3545',fontweight='bold')
    for ax in axs.flat:
        if ax.axison:ax.grid(alpha=.17);ax.set_axisbelow(True)
    fig.text(.065,.064,'Stop intervals exclude cleanup STOP. Hits = collision callbacks. A pass also requires no collision and successful cleanup.',fontsize=9,color='#475569')
    fig.text(.065,.038,'Ground-truth input verifies planning/control behavior; it does not demonstrate improved LiDAR detection. XY axes have independent scales.',fontsize=9,color='#64748b')
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=160,facecolor='white');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    default=Path(__file__).resolve().parents[1]/'docs/validation/carla-behavior-repair-20260922'
    parser.add_argument('--input-dir',type=Path,default=default)
    parser.add_argument('--before-dir',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    # Auxiliary *_run1_aeb.json may be an actively written observer stream;
    # setup attempts have their own archive and are not final repeat trials.
    trial_name = re.compile(r'^(baseline|vehicle_stop|vehicle_avoidance|pedestrian_stop)_run[0-9]+\.json$')
    paths=sorted((p for p in args.input_dir.glob('*_run*.json') if trial_name.fullmatch(p.name)),
                 key=lambda p:(CASES.index(p.stem.rsplit('_run',1)[0]),int(p.stem.rsplit('_run',1)[1])))
    if not paths:parser.error('No CASE_runN.json final trial records found')
    reports={p.stem:json.loads(p.read_text()) for p in paths}
    reports={k:r for k,r in reports.items() if r.get('case') in CASES}
    if not reports:parser.error('No recognized trial case records found')
    metrics={p.stem:analyse_one(p,reports[p.stem]) for p in paths if p.stem in reports}
    before={}
    if args.before_dir:
        for case in CASES:
            path=args.before_dir/(case+'.json')
            if path.is_file():before[case]=json.loads(path.read_text())
    output=args.output or args.input_dir/'summary.png'
    plot(reports,metrics,before,output)
    analysis={'stop_threshold_mps':STOP_MPS,'timing_note':'All intervals use CARLA sample/frame time. ROS object age uses ROS time only. Raw files are unchanged.',
              'pass_label_note':'Requires reported harness success, independently observed physical criteria and verified cleanup. Collision callbacks are contacts within one trial, not independent trials.',
              'trials':metrics,'counts':dict(Counter(m['verdict'] for m in metrics.values()))}
    output.with_name('analysis.json').write_text(json.dumps(analysis,indent=2)+'\n')
    print(json.dumps({'figure':str(output),'counts':analysis['counts'],'trials':[
        {k:m[k] for k in ('run','verdict','observed_peak_kmh','min_clearance_m','final_clearance_m','max_lateral_m','collision_callback_count','longest_stop_before_cleanup_s','continued_after_pass_s')}
        for m in metrics.values()]},indent=2))


if __name__=='__main__':main()
