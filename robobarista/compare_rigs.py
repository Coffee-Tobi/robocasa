"""Compare side-marker camera rigs with identical seeded MuJoCo trials.

Run in a Colab CPU runtime after installing requirements. This is NOT a calibration
of real cameras or a dynamic latte-art accuracy certification.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from pathlib import Path

from .benchmark_gates import summarize_existing_run


def analyze(args):
    root=Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=True)
    backend='egl' if os.path.exists('/dev/nvidia0') else 'osmesa'
    env=os.environ.copy()
    env['MUJOCO_GL']=backend
    if backend=='osmesa':
        env['PYOPENGL_PLATFORM']='osmesa'
    rows=[]
    for pose_method in ('bundle_adjust','caliscope'):
        for method in ('opencv','pupil'):
            for tag_count in (1,2):
                for camera_count in (3,4):
                    label=f'{pose_method}_{method}_{tag_count}tag_{camera_count}cam'
                    folder=root/label
                    command=[sys.executable,'-u','-m','robobarista.demo',
                        '--pitcher-side-tags','--pitcher-tags',str(tag_count),
                        '--cameras',str(camera_count),'--detector',method,
                        '--pose-method',pose_method,
                        '--trials',str(args.trials),'--seed',str(args.seed),
                        '--width',str(args.width),'--height',str(args.height),
                        '--output',str(folder),'--min-cameras','2',
                        '--max-spout-mm','0.5',
                        '--max-pitcher-rotation-deg','1.0']
                    print('\nTesting',label,'| backend',backend,flush=True)
                    result=subprocess.run(command,env=env,text=True,
                                          stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE,errors='replace')
                    (root/f'{label}.stdout.log').write_text(result.stdout)
                    (root/f'{label}.stderr.log').write_text(result.stderr)
                    summary_path=folder/'summary.json'
                    summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
                    if summary and (folder/'measurements.json').exists():
                        _,gate=summarize_existing_run(folder)
                    else:
                        gate={'pitcher_status':'ERROR','pitcher_failures':[],
                              'other_failures':[]}
                    row={
                        'variant':label,'detector':method,'pose_method':pose_method,
                        'pitcher_tags':tag_count,'cameras':camera_count,
                        'status':summary.get('status','ERROR'),
                        'pitcher_status':gate['pitcher_status'],
                        'pitcher_failures':gate['pitcher_failures'],
                        'other_failures':gate['other_failures'],
                        'exit_code':result.returncode,
                        'pitcher_valid_trials':summary.get('pitcher_valid_trials',0),
                        'trials':args.trials,
                        'min_pitcher_cameras_seen':summary.get('min_pitcher_cameras_seen',0),
                        'max_spout_tip_mm':summary.get('max_spout_tip_error_mm'),
                        'max_pitcher_angle_deg':summary.get('max_pitcher_rotation_deg'),
                        'max_relative_spout_to_cup_mm':summary.get('max_relative_tip_to_cup_error_mm'),
                        'missing_measurements':len(summary.get('missing_or_insufficient_views',[])),
                        'runtime_seconds':summary.get('duration_s'),
                        'error_excerpt':result.stderr[-1500:] if not summary else ''
                    }
                    rows.append(row)
                    print('  scene:',row['status'],'| pitcher:',row['pitcher_status'],'| measured',
                          row['pitcher_valid_trials'],'/',args.trials,
                          '| max spout mm:',row['max_spout_tip_mm'],
                          '| max pitcher angle:',row['max_pitcher_angle_deg'],
                          '| min cameras:',row['min_pitcher_cameras_seen'])
                    if row['status']=='ERROR':
                        print('  crash:',row['error_excerpt'][-700:])
                    (root/'comparison.json').write_text(json.dumps(rows,indent=2))
    fields=list(rows[0])
    with (root/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    pitcher_candidates=[r for r in rows if r['pitcher_status']=='PASS'
                        and r['pitcher_valid_trials']==args.trials]
    scene_candidates=[r for r in pitcher_candidates if r['status']=='PASS']
    for label,candidates in [('Pitcher-only precision',pitcher_candidates),
                             ('Entire scene including other objects',scene_candidates)]:
        if candidates:
            winner=min(candidates,key=lambda r:(
                r['max_spout_tip_mm'],r['max_pitcher_angle_deg'],
                r['cameras'],r['pitcher_tags']))
            print('\nBest for',label+':',winner['variant'],
                  '| spout mm',winner['max_spout_tip_mm'])
        else:
            print('\nNo passing configuration for',label)
    print('Remember: only three static ideal-camera poses, not real-sensor validation.')
    print('Comparison and logs saved in',root)
    return rows


def analyze_existing(args):
    """Re-evaluate existing output without costly repeat MuJoCo rendering."""
    root=Path(args.output).resolve()
    rows=[]
    for summary_path in sorted(root.glob('*/summary.json')):
        folder=summary_path.parent
        try:
            summary,gate=summarize_existing_run(folder)
        except (ValueError,OSError,KeyError,TypeError) as exc:
            print('Skipping',folder.name,':',exc)
            continue
        row=dict(variant=folder.name,scene_status=gate['scene_status'],
                 pitcher_status=gate['pitcher_status'],
                 pitcher_valid_trials=summary.get('pitcher_valid_trials'),
                 max_spout_tip_mm=summary.get('max_spout_tip_error_mm'),
                 max_pitcher_angle_deg=summary.get('max_pitcher_rotation_deg'),
                 max_relative_spout_to_cup_mm=summary.get('max_relative_tip_to_cup_error_mm'),
                 pitcher_failures=gate['pitcher_failures'],
                 other_failures=gate['other_failures'])
        rows.append(row)
        print('\n',folder.name,
              '| pitcher:',gate['pitcher_status'],'| scene:',gate['scene_status'],
              '| spout:',row['max_spout_tip_mm'],'mm')
        for issue in gate['pitcher_failures']+gate['other_failures']:
            print('  Failure:',issue)
    if not rows:
        print('No cached runs with summary.json + measurements.json in',root)
        return []
    (root/'diagnostics.json').write_text(json.dumps(rows,indent=2))
    with (root/'diagnostics.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    passing=[r for r in rows if r['pitcher_status']=='PASS'
             and r['max_spout_tip_mm'] is not None]
    if passing:
        best=min(passing,key=lambda r:r['max_spout_tip_mm'])
        print('\nBest pitcher measurement (not full-scene ranking):',
              best['variant'],best['max_spout_tip_mm'],'mm')
    print('Saved:',root/'diagnostics.csv')
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--analyze-only',action='store_true',help='Use existing saved runs, no rendering')
    parser.add_argument('--output',default='runs/rig_comparison')
    parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=960)
    options=parser.parse_args()
    if options.analyze_only:
        analyze_existing(options)
    else:
        analyze(options)
