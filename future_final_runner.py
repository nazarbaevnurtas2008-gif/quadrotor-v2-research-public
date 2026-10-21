"""FROZEN FUTURE 100-CONDITION RUNNER -- DO NOT EXECUTE DURING FREEZE AUDIT.
To run once AFTER explicit authorization:
 python future_final_runner.py --execute-final-once --confirm-sha256 <freeze_commit_hash>
Pre-registered sampling and analysis. Only outputs are mutable; source + controller hashes are sealed.
"""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.stats import binomtest
ROOT=Path(__file__).resolve().parent
SRC=ROOT/'frozen_source'
sys.path.insert(0,str(SRC))
import fair_retuning as fr
AUDIT=fr.audit
HORIZON=1201

def sha(b):return hashlib.sha256(b).hexdigest()

def assert_frozen(protocol):
    commitment=json.loads((ROOT/'SEED_COMMITMENT.json').read_text())
    actual=sha(json.dumps(commitment,sort_keys=True,separators=(',',':')).encode())
    assert actual==protocol['freeze_commit_hash_sha256'], 'Frozen manifest commitment changed.'
    for name, expected in protocol['sources_sha256'].items():
        assert sha((SRC/name).read_bytes())==expected, f'Source file changed: {name}'
    assert protocol['final_future_seed']==int(actual[:16],16)
    # Verify runner source is also included in commitment, except this does not require reading final data.
    expected_runner=protocol.get('future_final_runner_sha256')
    assert expected_runner and sha(Path(__file__).read_bytes())==expected_runner, 'Final runner changed'
    return True

def condition_draws(seed):
    # Must be called only from explicit future execution; freezes distributions and order.
    r=np.random.Generator(np.random.PCG64(seed))
    out=[]
    for i in range(100):
        out.append(dict(case_id=i+1,
          mass_kg=float(r.uniform(1.30,2.60)),horizontal_wind_mps=float(r.uniform(0.,7.5)),
          vertical_wind_mps=float(r.uniform(-2.5,2.5)),external_roll_torque_Nm=float(r.uniform(-.08,.08)),
          inertia_scale=float(r.uniform(.85,1.15)),motor_tau_scale=float(r.uniform(.8,1.25)),
          thrust_scale=float(r.uniform(.90,1.08)),drag_scale=float(r.uniform(.70,1.35)),
          noise_scale=float(r.uniform(.5,1.8)),delay_steps=int(r.integers(0,4))))
    return out

def noisy_measurements(seed,i,scale):
    # Independent paired noise stream for each scenario; same arrays across 4 controllers.
    noise_seed=int(sha(f'paired-noise:{seed}:{i}'.encode())[:16],16)
    rng=np.random.Generator(np.random.PCG64(noise_seed))
    return (rng.standard_normal(HORIZON)*.003*scale,
            rng.standard_normal(HORIZON)*np.deg2rad(.05*scale))

def read_specs(proto):
    p=proto['controller_parameters_frozen']
    specs=[]
    for cname in ['Carefully tuned conventional PID','Performance Auto-PID','Robust Auto-PID','Actuator-aware Integral LQR']:
        v=p[cname]
        if v['type']=='PID':specs.append((cname,3,np.asarray(v['altitude'],float),np.asarray(v['roll'],float)))
        else:specs.append((cname,2,np.asarray(v['altitude_K'],float),np.asarray(v['roll_K'],float)))
    return specs

def wilson(success,n,z=1.959963984540054):
    p=success/n;den=1+z*z/n
    mid=(p+z*z/(2*n))/den
    rad=z/den*np.sqrt(p*(1-p)/n+z*z/(4*n*n))
    return [float(mid-rad),float(mid+rad)]

def holm_adjust(pvals):
    d=len(pvals);ix=np.argsort(pvals); adj=np.zeros(d); running=0.
    for rank,k in enumerate(ix):
        running=max(running, float(pvals[k])*(d-rank));adj[k]=min(1.,running)
    return adj.tolist()

def analyze(df,seed):
    base='Carefully tuned conventional PID';comp=['Performance Auto-PID','Robust Auto-PID','Actuator-aware Integral LQR']
    s=[]
    for name,g in df.groupby('controller',sort=False):
        n=len(g);passn=int(g.altitude_pass.sum())
        s.append({'controller':name,'n':n,'altitude_task_pass':passn,'pass_rate':passn/n,
                  'wilson_ci_95':wilson(passn,n),
                  'mean_settle_s_report_censored_at_12s':float(g.settling_s.mean()),
                  'not_settled_by_12s_count':int(g.horizon_censored.sum()),
                  'mean_altitude_IAE_m_s':float(g.altitude_IAE_m_s.mean()),
                  'mean_roll_IAE_deg_s':float(g.roll_IAE_deg_s.mean()),
                  'max_horizontal_drift_m':float(g.max_horizontal_drift_m.max()),
                  'max_abs_unwrapped_yaw_deg':float(g.max_abs_unwrapped_yaw_deg.max()),
                  'mean_actuator_saturation_pct':float(g.saturation_pct.mean())})
    paired=[];rng=np.random.Generator(np.random.PCG64(seed ^ 0xA53D72231795F05A))
    B=df[df.controller==base].set_index('case_id').sort_index()
    pvals=[]
    for target in comp:
        X=df[df.controller==target].set_index('case_id').sort_index()
        assert list(X.index)==list(B.index)
        b=int((B.altitude_pass & ~X.altitude_pass).sum())
        c=int((~B.altitude_pass & X.altitude_pass).sum())
        p=float(binomtest(min(b,c),b+c,0.5,alternative='two-sided').pvalue) if (b+c)>0 else 1.
        pvals.append(p)
        effects={}
        for m in ['settling_s','altitude_IAE_m_s','roll_IAE_deg_s','max_abs_unwrapped_yaw_deg','max_horizontal_drift_m','saturation_pct']:
            d=(X[m]-B[m]).to_numpy(float)
            ids=rng.integers(0,len(d),size=(10000,len(d)))
            bt=np.mean(d[ids],axis=1)
            effects[m]={'paired_mean_difference_target_minus_baseline':float(d.mean()),
                'bootstrap_CI_95':np.quantile(bt,[.025,.975]).tolist()}
        paired.append({'comparison':f'{target} minus {base}','baseline_only_success':b,
                       'target_only_success':c,'mcnemar_exact_two_sided_p':p,'continuous_metrics':effects})
    adj=holm_adjust(pvals)
    for j,r in enumerate(paired):r['mcnemar_holm_adjusted_p']=adj[j]
    return {'summary':s,'paired_planned':paired,
            'statistical_scope':'100 independent synthetic draws shared by all controllers; not empirical flight-safety inference'}

def main():
    a=argparse.ArgumentParser();a.add_argument('--execute-final-once',action='store_true');a.add_argument('--confirm-sha256');args=a.parse_args()
    protocol=json.loads((ROOT/'FROZEN_FINAL_PROTOCOL.json').read_text());assert_frozen(protocol)
    if not args.execute_final_once or args.confirm_sha256!=protocol['freeze_commit_hash_sha256']:
        print('SEALED: controller and study code verified. Final 100-condition data NOT generated. Explicit authorization required.')
        return
    out=ROOT/'FINAL_EXPERIMENT_RESULTS'
    if out.exists():raise RuntimeError('One-shot final experiment directory already exists. Never re-run to cherry-pick results.')
    out.mkdir()
    seed=protocol['final_future_seed'];conds=condition_draws(seed)
    pd.DataFrame(conds).to_csv(out/'final_conditions_FROZEN.csv',index=False)
    results=[]
    for cond in conds:
        case=np.asarray([cond['mass_kg'],8.,cond['vertical_wind_mps'],cond['horizontal_wind_mps'],cond['external_roll_torque_Nm'],3.,5.],float)
        nz,nr=noisy_measurements(seed,cond['case_id'],cond['noise_scale'])
        for cname,mode,alt,roll in read_specs(protocol):
            z,r,u,sat,xy,pitch,yaw,tgap,rgap=AUDIT.run_measured(case,alt,roll,mode,nz,nr,cond['delay_steps'],1,
                mot_sc=cond['motor_tau_scale'],drag_sc=cond['drag_scale'],in_sc=cond['inertia_scale'],thrust_sc=cond['thrust_scale'])
            metric=AUDIT.evaluate((z,r,u,sat,xy,pitch,yaw,tgap,rgap))
            if not np.all(np.isfinite(z)) or not np.all(np.isfinite(yaw)):
                # Never silently exclude failures from analysis. Mark numerical failure as altitude FAIL.
                altitude_pass=False
                metric={k:(v if np.isfinite(v) else np.nan) for k,v in metric.items() if isinstance(v,(int,float,np.number))}
                yaw_unwrap=np.unwrap(np.nan_to_num(yaw))
            else:
                altitude_pass=bool(metric['passes'])
                yaw_unwrap=np.unwrap(yaw)
            target=2.;outside=np.flatnonzero(np.abs(z-target)>.05*target) if np.all(np.isfinite(z)) else np.array([len(z)-1])
            censored=bool(len(outside) and outside[-1]>=len(z)-1)
            results.append({'case_id':cond['case_id'],'controller':cname,'altitude_pass':bool(altitude_pass),
               'settling_s':float(metric.get('settling_s',np.nan)),'horizon_censored':censored,
               'overshoot_pct':float(metric.get('overshoot_pct',np.nan)),
               'altitude_IAE_m_s':float(metric.get('IAE',np.nan)),
               'roll_IAE_deg_s':float(metric.get('roll_IAE_deg_s',np.nan)),
               'max_abs_roll_deg':float(np.rad2deg(np.nanmax(np.abs(r)))),
               'max_horizontal_drift_m':float(np.nanmax(xy)),
               'max_abs_pitch_deg':float(np.rad2deg(np.nanmax(np.abs(pitch)))),
               'max_abs_unwrapped_yaw_deg':float(np.rad2deg(np.max(np.abs(yaw_unwrap)))),
               'net_unwrapped_yaw_deg':float(np.rad2deg(yaw_unwrap[-1]-yaw_unwrap[0])),
               'thrust_increment_RMS_N':float(metric.get('uRMS_N',np.nan)),
               'saturation_pct':float(metric.get('saturation_pct',np.nan)),
               'max_collective_error_N':float(metric.get('max_abs_T_command_gap_N',np.nan)),
               'max_roll_allocation_error_Nm':float(metric.get('max_abs_roll_command_gap_Nm',np.nan)),
               'final_altitude_error_m':float(metric.get('final_error_m',np.nan)),
               'yaw_over_45deg_diagnostic':bool(np.max(np.abs(yaw_unwrap))>np.deg2rad(45.)),
               'xy_over_2m_diagnostic':bool(np.max(xy)>2.)})
    frame=pd.DataFrame(results);frame.to_csv(out/'final_400_paired_controller_trials.csv',index=False)
    analysis=analyze(frame,seed)
    (out/'statistics_pre_registered.json').write_text(json.dumps(analysis,indent=2))
    (out/'execution_manifest.json').write_text(json.dumps({'freeze_hash':protocol['freeze_commit_hash_sha256'],
        'final_seed':seed,'N':100,'controllers':4,'final_trial_rows':len(frame),
        'conditions_sha256':sha((out/'final_conditions_FROZEN.csv').read_bytes())},indent=2))
    print('FINAL one-shot experiment complete. Do NOT retune. Saved',out)
if __name__=='__main__':main()
