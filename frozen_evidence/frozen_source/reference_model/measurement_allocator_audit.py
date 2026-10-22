"""Development-only measurement fairness + allocation audit for Nurtas Nazarbayev's
reconstructed quadrotor V2 model. No final held-out samples generated.

This code imports the already-produced plant integration and controller conventions
from ../quadrotor_v2_development/v2_lqr_development.py. It does NOT modify gains.
Development dataset was already involved in tuning: this is a diagnostic audit,
not external validation.
"""
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd
from numba import njit
P=Path(__file__).resolve().parent
PRIOR=P if (P/'v2_lqr_development.py').exists() else ((P/'reference_model') if (P/'reference_model'/'v2_lqr_development.py').exists() else (P.parent/'quadrotor_v2_development'))
sys.path.insert(0,str(PRIOR))
import v2_lqr_development as m

@njit(cache=True)
def allocate_priority(Tcmd,txcmd):
    """Pitch and yaw set to zero; collective preserved, roll limited by available headroom.
    Configuration f0=f2=T/4, f1=T/4+d, f3=T/4-d, tx=2*arm*d.
    """
    reqT=max(0.,min(48.,Tcmd))
    center=.25*reqT
    max_d=min(center,12.-center)
    d=max(-max_d,min(max_d,txcmd/(2.*m.ARM)))
    f=np.array([center,center+d,center,center-d])
    return np.sqrt(f/m.KF),f

@njit(cache=True)
def run_measured(case,kz,kr,mode,noise_z,noise_roll,delay_steps=0,
                 allocator_type=0,nsteps=1201,mot_sc=1.,drag_sc=1.,in_sc=1.,thrust_sc=1.,zref=2.):
    """Same control law and 6DOF plant as v2_lqr_development.run_sim.
    - noise arrays are prepared IN ADVANCE using one shared seed per condition/replicate
    - delay uses ONLY already measured data; no future state leakage
    - same z,roll observations and same filtered derivative for every controller
    - rotor state used for simulation and metrics only, never fed into feedback
    - only changed alloc option preserves the requested collective when roll clips
    """
    mass, roll0, wz, wx, moment,t_start,t_end=case
    x=np.zeros(17)
    x[6]=np.cos(np.deg2rad(roll0)/2.)
    x[7]=np.sin(np.deg2rad(roll0)/2.)
    x[13:17]=np.sqrt(m.M_NOM*m.G/(4*m.KF))
    zarr=np.zeros(nsteps);rarr=np.zeros(nsteps);uarr=np.zeros(nsteps)
    saturation=np.zeros(nsteps)
    xy=np.zeros(nsteps); pitch=np.zeros(nsteps); yaw=np.zeros(nsteps)
    alloc_error_T=np.zeros(nsteps);alloc_error_tau=np.zeros(nsteps)
    obs_z_arr=np.zeros(nsteps);obs_r_arr=np.zeros(nsteps)
    last_z=0.;last_roll=m.roll_from_quat(x[6:10])
    zrate_hat=0.;rrate_hat=0.
    iz=0.;ir=0.;act_hat_T=0.;act_hat_roll=0.
    integral_z=0.;integral_roll=0.
    if mode==0:
        alt_pid=(14.,3.,6.);roll_pid=(.9,.1,.26)
    elif mode==1:
        alt_pid=(24.,9.,8.5);roll_pid=(1.7,0.,.36)
    elif mode==3:
        alt_pid=(kz[0],kz[1],kz[2]);roll_pid=(kr[0],kr[1],kr[2])
    else:
        alt_pid=(0.,0.,0.);roll_pid=(0.,0.,0.)
    for k in range(nsteps):
        t=k*m.DT
        z=x[2];roll=m.roll_from_quat(x[6:10])
        zarr[k]=z;rarr[k]=roll
        xy[k]=np.sqrt(x[0]*x[0]+x[1]*x[1])
        q0,q1,q2,q3=x[6:10]
        sinp=2*(q0*q2-q3*q1)
        pitch[k]=np.arcsin(max(-1.,min(1.,sinp)))
        yaw[k]=np.arctan2(2*(q0*q3+q1*q2),1.-2*(q2*q2+q3*q3))
        obs_z_arr[k]=z+noise_z[k]
        obs_r_arr[k]=roll+noise_roll[k]
        idx=max(0,k-delay_steps)
        zm=obs_z_arr[idx];rm=obs_r_arr[idx]
        alpha=m.DT/(m.FILTER_TAU+m.DT)
        zrate_hat+=alpha*((zm-last_z)/m.DT-zrate_hat)
        rrate_hat+=alpha*((rm-last_roll)/m.DT-rrate_hat)
        last_z=zm;last_roll=rm
        errz=zref-zm
        errr=-rm
        if mode==2:
            dz_cmd=-(kz[0]*(-errz) + kz[1]*zrate_hat + kz[2]*act_hat_T + kz[3]*iz)
            dr_cmd=-(kr[0]*rm + kr[1]*rrate_hat + kr[2]*act_hat_roll + kr[3]*ir)
            rawT=m.M_NOM*m.G+dz_cmd
            rawR=dr_cmd
            cmdT=max(0.,min(48.,rawT))
            cmdR=max(-4.,min(4.,rawR))
            if not ((rawT>48. and errz>0.) or (rawT<0. and errz<0.)):
                iz+=m.DT*(-errz)
            if not ((rawR>4. and errr>0.) or (rawR< -4. and errr<0.)):
                ir+=m.DT*(-errr)
            iz=max(-10.,min(10.,iz));ir=max(-2.,min(2.,ir))
            # Actuator prediction nominal only, no truth feedback
            act_hat_T+=m.DT*((cmdT-m.M_NOM*m.G)-act_hat_T)/m.TAU_M
            act_hat_roll+=m.DT*(cmdR-act_hat_roll)/m.TAU_M
        else:
            kp,ki,kd=alt_pid
            krp,kri,krd=roll_pid
            unsat_z=kp*errz+ki*integral_z-kd*zrate_hat
            sat_z=max(-m.M_NOM*m.G,min(48.-m.M_NOM*m.G,unsat_z))
            unsat_r=krp*errr+kri*integral_roll-krd*rrate_hat
            sat_r=max(-4.,min(4.,unsat_r))
            integral_z=max(-20.,min(20.,integral_z+m.DT*(errz+m.KAW*(sat_z-unsat_z))))
            integral_roll=max(-2.,min(2.,integral_roll+m.DT*(errr+m.KAW*(sat_r-unsat_r))))
            cmdT=m.M_NOM*m.G+sat_z;cmdR=sat_r
        if allocator_type==0:
            oc,f=m.allocate(cmdT,cmdR)
        else:
            oc,f=allocate_priority(cmdT,cmdR)
        alloc_error_T[k]=np.sum(f)-max(0.,min(48.,cmdT))
        alloc_error_tau[k]=m.ARM*(f[1]-f[3])-cmdR
        uarr[k]=cmdT-m.M_NOM*m.G
        saturation[k]=1. if (np.min(f)<1e-6 or np.max(f)>=12.-1e-6) else 0.
        gust=(t>=t_start and t<=t_end)
        x=m.step_rk4(x,oc,mass,wx if gust else 0.,wz if gust else 0.,moment if gust else 0.,mot_sc,drag_sc,in_sc,thrust_sc)
        if not np.all(np.isfinite(x)) or np.max(np.abs(x))>1.e5:
            zarr[k:]=np.nan;rarr[k:]=np.nan
            xy[k:]=np.nan;pitch[k:]=np.nan;yaw[k:]=np.nan
            break
    return zarr,rarr,uarr,saturation,xy,pitch,yaw,alloc_error_T,alloc_error_tau


def evaluate(out,zref=2.):
    z,roll,u,sat,xy,pitch,yaw,tgap,rgap=out
    result=m.metrics(z,roll,u,sat,zref=zref)
    result.update(max_xy_drift_m=float(np.nanmax(xy)),max_abs_pitch_deg=float(np.rad2deg(np.nanmax(np.abs(pitch)))),max_abs_yaw_deg=float(np.rad2deg(np.nanmax(np.abs(yaw)))),
        max_abs_T_command_gap_N=float(np.nanmax(np.abs(tgap))),rms_T_command_gap_N=float(np.sqrt(np.nanmean(tgap*tgap))),
        max_abs_roll_command_gap_Nm=float(np.nanmax(np.abs(rgap))))
    return result


def controller_specs():
    st=json.loads((PRIOR/'stage2_summary.json').read_text())
    pf=json.loads((PRIOR/'performance_10seed_summary.json').read_text())
    rb=json.loads((PRIOR/'robust_dev_summary.json').read_text())
    pr=pf['selected_gains']
    return [
        ('Tuned PID',1,np.zeros(4),np.zeros(4)),
        ('Performance Auto-PID',3,np.array([pr['alt_Kp'],pr['alt_Ki'],pr['alt_Kd']]),np.array([pr['roll_Kp'],pr['roll_Ki'],pr['roll_Kd']])),
        ('Robust Auto-PID',3,np.array(rb['selected_altitude_K']),np.array(rb['roll_K'])),
        ('Actuator-aware Integral LQR',2,np.array(st['Kz']),np.array(st['Kroll']))
    ]


def validate_allocators():
    Ts=[0.,1.,5.,10.,14.715,25.,40.,47.,48.,55.]
    torques=[-5.,-4.,-2.,-.5,0.,.5,2.,4.,5.]
    rows=[]
    for T in Ts:
        for torque in torques:
            for label,fn in [('existing independent clipping',m.allocate),('collective-priority corrected',allocate_priority)]:
                om,f=fn(T,torque)
                rows.append({'algorithm':label,'requested_T_N':T,'requested_roll_torque_Nm':torque,'achieved_T_N':float(np.sum(f)),
                             'achieved_roll_torque_Nm':float(m.ARM*(f[1]-f[3])),
                             'thrust_error_N':float(np.sum(f)-np.clip(T,0,48)),
                             'roll_torque_error_Nm':float(m.ARM*(f[1]-f[3])-torque),
                             'valid_rotor_bounds':bool(np.all(f>=-1e-12) and np.all(f<=12.+1e-12) and np.all(om>=-1e-12) and np.all(om<=m.OMEGA_MAX+1e-9))})
    df=pd.DataFrame(rows);df.to_csv(P/'allocator_invariant_grid.csv',index=False)
    return df


def main():
    specs=controller_specs()
    # Zero-noise exact-match regression before any noisy trials
    zeros=np.zeros(1201)
    eq=[]
    for name,mode,kz,kr in specs:
        for idx,c in enumerate(m.DEV_CASES,1):
            old=m.run_sim(c,kz,kr,mode)
            new=run_measured(c,kz,kr,mode,zeros,zeros,0,0)
            err=max(float(np.max(np.abs(a-b))) for a,b in zip(old,new[:4]))
            eq.append(dict(controller=name,scenario=f'D{idx}',max_abs_value_deviation=err,equivalent=bool(err<1.e-10)))
    eqdf=pd.DataFrame(eq);eqdf.to_csv(P/'zero_noise_regression.csv',index=False)
    if not eqdf.equivalent.all():
        raise RuntimeError('Zero-noise implementation NOT equivalent to prior run_sim: see CSV')
    adf=validate_allocators()
    # Diagnostic scenarios only: existing 4 development D1-D4.
    # Noise and delay assumptions are NEW; no controller gains selected here.
    # Same pre-generated noise realization for every controller and allocator per paired trial.
    reps=[20261017051,20261017052,20261017053,20261017054,20261017055]
    factors=[1.,2.]
    delays=[0,1,3]
    base_sigma_z_m=.003
    base_sigma_roll_deg=.05
    results=[]
    for d_idx,c in enumerate(m.DEV_CASES,1):
        for rep in reps:
            # separate streams per scenario replicate; no requested/final scenarios sampled.
            rng=np.random.default_rng(rep + 101*d_idx)
            raw_z=rng.standard_normal(1201)
            raw_roll=rng.standard_normal(1201)
            for noise_scale in factors:
                nz=raw_z*base_sigma_z_m*noise_scale
                nr=raw_roll*np.deg2rad(base_sigma_roll_deg*noise_scale)
                for delay in delays:
                    for atype in (0,1):
                        for name,mode,kz,kr in specs:
                            out=run_measured(c,kz,kr,mode,nz,nr,delay,atype)
                            results.append(dict(scenario=f'D{d_idx}',rep_seed=rep,noise_scale=noise_scale,delay_ms=10*delay,
                                                allocation='independent_clip' if atype==0 else 'collective_priority',controller=name,**evaluate(out)))
    df=pd.DataFrame(results)
    df.to_csv(P/'measurement_delay_paired_trials.csv',index=False)
    agg=df.groupby(['allocation','controller','noise_scale','delay_ms'],as_index=False).agg(
       n=('passes','count'),passes=('passes','sum'),mean_settling_s=('settling_s','mean'),mean_altitude_IAE=('IAE','mean'),
       mean_saturation_pct=('saturation_pct','mean'),mean_uRMS_N=('uRMS_N','mean'),
       max_T_command_gap_N=('max_abs_T_command_gap_N','max'),mean_max_T_command_gap_N=('max_abs_T_command_gap_N','mean'),
       mean_max_xy_drift_m=('max_xy_drift_m','mean'),mean_max_pitch_deg=('max_abs_pitch_deg','mean'))
    agg['pass_pct']=100.*agg.passes/agg.n
    agg.to_csv(P/'measurement_delay_summary.csv',index=False)
    # Compared against current no-noise deterministic development using both allocators
    simple=[]
    for i,c in enumerate(m.DEV_CASES,1):
        for name,mode,kz,kr in specs:
            for a in (0,1):
                q=evaluate(run_measured(c,kz,kr,mode,zeros,zeros,0,a))
                simple.append(dict(scenario=f'D{i}',controller=name,allocation='independent_clip' if a==0 else 'collective_priority',**q))
    sdf=pd.DataFrame(simple);sdf.to_csv(P/'allocator_development_nonoise_comparison.csv',index=False)
    # output invariant checks
    checks={
      'zero_noise_regression_all_16_reproduced':bool(eqdf.equivalent.all()),
      'allocation_grid_all_rotors_bounded':bool(adf.valid_rotor_bounds.all()),
      'priority_grid_collective_thrust_error_near_zero':bool((adf[adf.algorithm=='collective-priority corrected'].thrust_error_N.abs()<1e-10).all()),
      'independent_clipping_can_break_collective_priority':bool((adf[adf.algorithm=='existing independent clipping'].thrust_error_N.abs()>1e-4).any()),
      'raw_trial_rows':len(df),'noise_trial_design':'D1-D4 x 5 paired noise seeds x 2 noise scales x 3 sample delays x 2 allocators x 4 unchanged controllers',
      'final_100_condition_heldout':'NOT GENERATED; NOT ACCESSED',
      'limitations':['development cases were used for tuning; not independent final evaluation',
          '2 active control loops: altitude and roll; 6DOF propagation is not full 6DOF stabilization',
          'physical motor speed dynamics modeled, real ESC/motor calibration unknown',
          'allocator correction changes simulation, so prior controller tuning and comparisons need re-run before any final freeze',
          'noise Gaussian iid and delay fixed within run are reconstruction assumptions']
    }
    (P/'audit_checks.json').write_text(json.dumps(checks,indent=2),encoding='utf8')
    if not all(checks[k] for k in list(checks)[:4]):raise RuntimeError('failed audit checks')
    print(json.dumps(checks,indent=2))
    print('\nPaired measurement summary noise scale 1 and delays 0 vs 30 (both allocators):')
    print(agg[(agg.noise_scale==1.) & (agg.delay_ms.isin([0,30]))][['allocation','controller','delay_ms','passes','n','mean_settling_s','mean_altitude_IAE','mean_max_T_command_gap_N']].to_string(index=False,float_format=lambda a:f'{a:.3f}'))
    print('\nAllocator nominal dev pass comparison:')
    print(sdf.groupby(['allocation','controller']).agg(passes=('passes','sum'),mean_settling=('settling_s','mean'),mean_Tgap=('max_abs_T_command_gap_N','mean'),mean_max_xy=('max_xy_drift_m','mean')).to_string(float_format=lambda a:f'{a:.3f}'))
    print('\nMax mixer grid collective error existing',adf[adf.algorithm=='existing independent clipping'].thrust_error_N.abs().max())

if __name__=='__main__':main()
