#!/usr/bin/env python3
"""Read-only verification of final frozen trajectories (NOT new sampling or re-tuning).
Replays existing final conditions and identical deterministic noise with source/hash checks.
Never writes to frozen source or original results. Adds missing max|z| evidence.
"""
import os,sys,hashlib,json,time
from pathlib import Path
import numpy as np,pandas as pd
ROOT=Path(os.environ.get('QUADROTOR_V2_FROZEN_ROOT', str(Path(__file__).resolve().parent/'frozen_evidence')))
if not ROOT.exists():
    ROOT=Path('/mnt/data/quadrotor_v2_final_analysis')
DEST=Path(__file__).parent
sys.path.insert(0,str(ROOT))
import future_final_runner as fn
protocol=json.loads((ROOT/'FROZEN_FINAL_PROTOCOL.json').read_text())
fn.assert_frozen(protocol)
original=pd.read_csv(ROOT/'final_400_paired_controller_trials.csv')
conds=pd.read_csv(ROOT/'final_conditions_FROZEN.csv')
seed=protocol['final_future_seed']
assert len(conds)==100 and len(original)==400
assert hashlib.sha256((ROOT/'final_conditions_FROZEN.csv').read_bytes()).hexdigest()==json.loads((ROOT/'execution_manifest.json').read_text())['conditions_sha256']
# Check that existing conditions correspond exactly to condition_draws, without generating new conditions
for rec,prev in zip(fn.condition_draws(seed),conds.to_dict(orient='records')):
    assert rec['case_id']==prev['case_id']
    for k,v in rec.items():
        assert np.isclose(v,prev[k],rtol=1e-12,atol=1e-12), (k,v,prev[k])

specs=fn.read_specs(protocol)
checks=[]
errors=[]
t0=time.time()
for n,cond in enumerate(conds.to_dict(orient='records')):
    case=np.array([cond['mass_kg'],8.,cond['vertical_wind_mps'],cond['horizontal_wind_mps'],cond['external_roll_torque_Nm'],3.,5.],float)
    nz,nr=fn.noisy_measurements(seed,cond['case_id'],cond['noise_scale'])
    for name,mode,alt,roll in specs:
        z,r,u,sat,xy,pitch,yaw,tgap,rgap=fn.AUDIT.run_measured(case,alt,roll,mode,nz,nr,int(cond['delay_steps']),1,
                            mot_sc=cond['motor_tau_scale'],drag_sc=cond['drag_scale'],in_sc=cond['inertia_scale'],thrust_sc=cond['thrust_scale'])
        metrics=fn.AUDIT.evaluate((z,r,u,sat,xy,pitch,yaw,tgap,rgap))
        row=original[(original.case_id==cond['case_id']) & (original.controller==name)].iloc[0]
        numeric_fields={
          'settling_s':metrics['settling_s'],
          'overshoot_pct':metrics['overshoot_pct'],
          'altitude_IAE_m_s':metrics['IAE'],
          'roll_IAE_deg_s':metrics['roll_IAE_deg_s'],
          'saturation_pct':metrics['saturation_pct'],
          'max_horizontal_drift_m':np.nanmax(xy),
          'max_abs_pitch_deg':np.rad2deg(np.nanmax(np.abs(pitch))),
          'max_abs_roll_deg':np.rad2deg(np.nanmax(np.abs(r))),
          'final_altitude_error_m':metrics['final_error_m'],
          'max_collective_error_N':np.nanmax(np.abs(tgap)),
          'max_roll_allocation_error_Nm':np.nanmax(np.abs(rgap)),
        }
        diff=max(abs(float(row[k])-float(v)) for k,v in numeric_fields.items())
        allfinite=bool(np.isfinite(z).all() and np.isfinite(r).all() and np.isfinite(yaw).all())
        maxz=float(np.nanmax(np.abs(z)))
        constraints={'numerically_finite':allfinite,'max_abs_roll_under_65_deg':bool(np.nanmax(np.abs(r))<np.deg2rad(65.)),
                     'max_abs_z_under_25_m':bool(maxz<25.),'pass_matches_replay':bool(metrics['passes'])==bool(row['altitude_pass']),
                     'saved_metrics_equal_replay':diff<1e-8}
        checks.append({'case_id':cond['case_id'],'controller':name,'max_abs_z_m':maxz,'min_z_m':float(np.nanmin(z)),
                       'max_z_m':float(np.nanmax(z)),'max_abs_roll_deg_replayed':float(np.rad2deg(np.nanmax(np.abs(r)))),
                       'max_row_replay_deviation':diff,**constraints})
        for k,flag in constraints.items():
            if not flag: errors.append((cond['case_id'],name,k))
    if (n+1)%20==0:
        print('progress',n+1,'/ 100 conditions, elapsed',round(time.time()-t0,1),'sec',flush=True)
df=pd.DataFrame(checks)
df.to_csv(DEST/'replayed_max_altitude_all_400.csv',index=False)
summary=df.groupby('controller',sort=False).agg(n=('case_id','count'),max_absolute_altitude_m=('max_abs_z_m','max'),
   min_altitude_m=('min_z_m','min'),max_altitude_m=('max_z_m','max'),replay_max_error=('max_row_replay_deviation','max'),
   abs_z_gate_pass_count=('max_abs_z_under_25_m','sum'),original_pass_agreement_count=('pass_matches_replay','sum'),
   exact_numeric_metric_replay_count=('saved_metrics_equal_replay','sum')).reset_index()
summary.to_csv(DEST/'replay_summary.csv',index=False)
report={'completed_replays':len(df),'elapsed_seconds':round(time.time()-t0,2),
        'overall_max_abs_z_m':float(df['max_abs_z_m'].max()),'overall_min_z_m':float(df['min_z_m'].min()),
        'altitude_guard_passes':int(df['max_abs_z_under_25_m'].sum()),'pass_agreement':int(df['pass_matches_replay'].sum()),
        'saved_metric_replay_passes':int(df['saved_metrics_equal_replay'].sum()),
        'failures':errors,'source_hash_verified':True,'original_results_modified':False,
        'note':'Exact replay of existing 100 frozen condition rows and shared noise, not a new held-out sample.'}
(DEST/'replay_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));print(summary.to_string(index=False))
