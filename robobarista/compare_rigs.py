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
                    row={
                        'variant':label,'detector':method,'pose_method':pose_method,
                        'pitcher_tags':tag_count,'cameras':camera_count,
                        'status':summary.get('status','ERROR'),
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
                    print('  status:',row['status'],'| measured',
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
    candidates=[r for r in rows if r['status']=='PASS'
                and r['pitcher_valid_trials']==args.trials
                and r['min_pitcher_cameras_seen']>=2]
    if candidates:
        # Strict pass first; only rank candidates meeting the full acceptance gate.
        winner=min(candidates,key=lambda r:(
            r['max_spout_tip_mm'],r['max_pitcher_angle_deg'],
            r['cameras'],r['pitcher_tags']))
        print('\nBest currently PASSING configuration by worst-case spout error:',
              winner['variant'])
        print('This is a small idealized simulation; not a real-world accuracy guarantee.')
    else:
        print('\nNo configuration passed all acceptance requirements.')
        print('Review side-tag image overlays, visibility, rotations and stderr logs.')
    print('Comparison and logs saved in',root)
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='runs/rig_comparison')
    parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=960)
    analyze(parser.parse_args())
