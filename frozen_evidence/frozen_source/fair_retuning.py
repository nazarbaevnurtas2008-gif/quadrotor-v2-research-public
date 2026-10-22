"""Development-only re-tuning after collective-priority allocator correction.
Nurtas Nazarbayev quadrotor V2; reconstructed model; not original source code.
This program DOES NOT generate/read any final 100-case test.
External S1--S3 known challenges are EXCLUDED from controller selection.
Four controllers share observed z, roll, filtered derivatives, rotor limits, 100Hz,
identical paired sensor noise and observation delay. The simulator is 6DOF with
only altitude and roll feedback; lateral/yaw position stabilization is absent.
"""
import os,sys,json,time,hashlib
from pathlib import Path
from functools import lru_cache
import numpy as np
import pandas as pd
from scipy.optimize import differential_evolution, minimize
ROOT=Path(__file__).resolve().parent
REF=ROOT/'reference_model'
sys.path.insert(0,str(REF))
import measurement_allocator_audit as audit
m=audit.m
import importlib

# Planned development & gate cases; no 3.0 kg. Each has nominally model-unseen conditions.
# mass, initial roll(deg), wz, wx, tau_x, gust start, gust end
TRAIN=[
 ('D1', (1.50,5.,0.,0.,0.,4.,5.), (1.,1.,1.,1.)),
 ('D2', (1.70,8.,-.8,3.,.025,4.,5.), (1.,1.,1.,1.)),
 ('D3', (2.20,5.,0.,0.,0.,4.,5.), (1.,1.,1.,1.)),
 ('D4', (1.40,7.,-1.,4.,-.020,4.,5.), (1.,1.,1.,1.)),
 ('D5', (2.35,9.,-1.2,5.,.050,3.5,5.5), (1.18,1.2,1.07,.93)),
 ('D6', (1.30,10.,-.8,5.5,-.055,4.,6.), (.85,.8,.9,1.06)),
]
# Gate is not held-out final: inspected for development diagnostics, no gains edited using it.
GATE=[
 ('G1',(1.92,6.,-1.1,4.5,.035,3.5,5.5), (1.08,1.05,1.02,.97)),
 ('G2',(2.48,9.,-1.4,5.7,-.055,4.,5.), (1.23,.75,1.1,.91)),
 ('G3',(2.05,10.,1.0,6.0,.07,4.,6.), (.9,1.25,.85,1.04)),
 ('G4',(1.45,5.,-.3,2.5,-.025,3.,5.), (1.10,1.0,1.0,.96)),
]
TUNE_SEEDS=[761001,761002]
TUNE_DELAYS=[0,3]
GATE_SEEDS=[990001,990002,990003]
GATE_DELAYS=[0,1,3]
NOISE_SIGMA_Z=.003
NOISE_SIGMA_ROLL_DEG=.05
N=1201
PREV={
 'tuned_alt':[24.,9.,8.5], 'tuned_roll':[1.7,0.,.36],
 'performance_alt':[25.589875313695522,.05010559272949351,9.116545662058257],
 'performance_roll':[1.868626065983877,.025562038497953764,.3351637138970239],
 'robust_alt':[26.444444444444443,14.,11.4], 'robust_roll':[1.7,0.,.36],
 'lqr_qz':[300.,10.,.01,12.], 'lqr_rz':.15,
 'lqr_qr':[200.,5.,.01,2.], 'lqr_rr':.8,
}

@lru_cache(None)
def noise_for(label,seed,sc):
    # pair same arrays across controllers. RNG parameters independent of evaluation sequence.
    # salt deterministic; not platform-dependent built-in hash().
    v=int.from_bytes(hashlib.sha256(label.encode('ascii')).digest()[:4],'little')
    rng=np.random.default_rng((seed+v)%(2**32))
    return rng.standard_normal(N)*NOISE_SIGMA_Z*sc, rng.standard_normal(N)*np.deg2rad(NOISE_SIGMA_ROLL_DEG)*sc

@lru_cache(None)
def experiment_cases(phase):
    base=TRAIN if phase=='train' else GATE
    seeds=TUNE_SEEDS if phase=='train' else GATE_SEEDS
    delays=TUNE_DELAYS if phase=='train' else GATE_DELAYS
    cases=[]
    for name,phys,unc in base:
        for seed in seeds:
            nz,nr=noise_for(name,seed,1.)
            for delay in delays:
                cases.append((name,int(seed),delay,np.array(phys,dtype=float),unc,nz,nr))
    return tuple(cases)

def run_all(mode,alt,roll,phase='train',details=False):
    rows=[]
    for label,seed,delay,cond,sc,nz,nr in experiment_cases(phase):
        try:
            out=audit.run_measured(cond,np.asarray(alt,dtype=float),np.asarray(roll,dtype=float),mode,nz,nr,delay,1,
                                   mot_sc=sc[0],drag_sc=sc[1],in_sc=sc[2],thrust_sc=sc[3])
            v=audit.evaluate(out)
            if not np.isfinite(v['IAE']):
                v=dict(stable=False,settling_s=12,overshoot_pct=500.,IAE=100.,roll_IAE_deg_s=500.,final_error_m=10.,saturation_pct=100.,uRMS_N=100.,passes=False)
        except Exception as exc:
            raise RuntimeError(f'Simulation failed: {phase}/{label}/seed={seed}/delay={delay*10}ms/mode={mode}') from exc
        rows.append({'case':label,'noise_seed':seed,'delay_ms':delay*10,'mode':mode,**v})
    return rows

def base_loss(row):
    return (.60*row['IAE']+.25*row['settling_s']+.016*row['overshoot_pct']+
            .012*row['saturation_pct']+.003*row['uRMS_N']+
            .025*row['roll_IAE_deg_s']+
            10.*max(0.,abs(row['final_error_m'])-.085)+
            .5*max(0.,row['overshoot_pct']-14.)+
            .8*max(0.,row['settling_s']-5.8)+
            5.*(not row['stable']))

def loss(rows,kind):
    vals=np.array([base_loss(r) for r in rows])
    pass_fr=np.mean([r['passes'] for r in rows]);
    if kind=='robust':
        tail=sorted(vals)[-max(1,int(np.ceil(.25*len(vals)))):]
        return float(np.mean(vals)+.7*np.mean(tail)+22.*(1.-pass_fr)**2)
    if kind=='performance':
        # Performance optimization emphasizes low mean error/settling; pass gate penalty nonzero.
        return float(np.mean(vals)+2.5*(1.-pass_fr)**2)
    return float(np.mean(vals)+5.*(1.-pass_fr)**2)

def pid_obj(gains,kind):
    g=np.array(gains,dtype=float)
    if (g[0]<8 or g[0]>32 or g[1]<0 or g[1]>16 or g[2]<3 or g[2]>16 or
        g[3]<.5 or g[3]>3.2 or g[4]<0 or g[4]>.3 or g[5]<.12 or g[5]>.7):
        return 1e5
    return loss(run_all(3,g[:3],g[3:],'train'),kind)

# Explicit predeclared weights (two-stage grid, K selected strictly on TRAIN)
QZ_GRID=[(100.,10.,.01,6.),(200.,10.,.01,10.),(300.,10.,.01,12.),
         (120.,20.,.03,15.),(200.,20.,.1,18.),(300.,20.,.05,12.),
         (150.,30.,.1,15.),(300.,30.,.1,20.)]
RZ_GRID=[.15,.3,.8]
QR_GRID=[(60.,5.,.01,1.),(100.,5.,.01,2.),(200.,5.,.01,2.),
         (120.,10.,.05,4.),(200.,10.,.1,5.),(100.,15.,.1,5.),
         (200.,15.,.1,8.),(250.,20.,.2,10.)]
RR_GRID=[.6,.8,2.,5.]

def tune_lqr():
    start=time.time()
    first_q=PREV['lqr_qr'];first_r=PREV['lqr_rr'];ref_roll=m.lqi_gain('roll',first_q,first_r)
    rows=[]
    for i,q in enumerate(QZ_GRID):
        for r in RZ_GRID:
            K=m.lqi_gain('alt',q,r)
            score=loss(run_all(2,K,ref_roll,'train'),'conventional')
            rows.append(dict(Qz=q,Rz=r,K=K.tolist(),score=score))
    rows.sort(key=lambda x:(x['score'],str(x['Qz']),x['Rz']))
    pd.DataFrame(rows).to_csv(ROOT/'lqr_altitude_training_grid.csv',index=False)
    kz=np.array(rows[0]['K'])
    rr=[]
    for i,q in enumerate(QR_GRID):
        for r in RR_GRID:
            K=m.lqi_gain('roll',q,r)
            score=loss(run_all(2,kz,K,'train'),'conventional')
            rr.append(dict(Qr=q,Rr=r,K=K.tolist(),score=score))
    rr.sort(key=lambda x:(x['score'],str(x['Qr']),x['Rr']))
    pd.DataFrame(rr).to_csv(ROOT/'lqr_roll_training_grid.csv',index=False)
    kr=np.array(rr[0]['K'])
    obj=loss(run_all(2,kz,kr,'train'),'conventional')
    result={'altitude':rows[0],'roll':rr[0],'Kz':kz.tolist(),'Kr':kr.tolist(),'train_score':obj,'elapsed_s':time.time()-start}
    (ROOT/'lqr_selected.json').write_text(json.dumps(result,indent=2))
    print('LQR selected',json.dumps(result),flush=True)
    return result

def tune_pid_conventional():
    start=time.time()
    # deterministic coordinate grid, identical observations, same train cases as LQR and Auto-PID.
    prev=np.array(PREV['tuned_alt']+PREV['tuned_roll'],dtype=float)
    trials=[]
    for kp in [18.,22.,24.,26.,28.]:
        for ki in [6.,9.,12.,15.]:
            for kd in [7.,9.,11.,13.]:
                g=prev.copy();g[:3]=[kp,ki,kd]
                trials.append((pid_obj(g,'conventional'),g.copy()))
    trials.sort(key=lambda x:x[0]);g=trials[0][1].copy()
    roll_trials=[]
    for rp in [1.2,1.7,2.2,2.8]:
        for ri in [0.,.04,.12]:
            for rd in [.26,.36,.48,.6]:
                x=g.copy();x[3:]=[rp,ri,rd]
                roll_trials.append((pid_obj(x,'conventional'),x.copy()))
    roll_trials.sort(key=lambda x:x[0]);g=roll_trials[0][1].copy()
    # local deterministic Powell with fixed bounds, maxfev strictly fixed; no gate used
    opt=minimize(lambda x:pid_obj(x,'conventional'),g,method='Powell',
                 bounds=[(12,32),(1,16),(5,16),(.6,3.2),(0,.3),(.18,.7)],
                 options={'maxfev':150,'xtol':.04,'ftol':.005,'disp':False})
    if opt.fun<roll_trials[0][0]:g=opt.x.copy(); score=float(opt.fun)
    else:score=roll_trials[0][0]
    out={'gains':g.tolist(),'training_score':score,'altitude_grid_n':len(trials),'roll_grid_n':len(roll_trials),'Powell_nfev':int(opt.nfev),'elapsed_s':time.time()-start}
    (ROOT/'conventional_selected.json').write_text(json.dumps(out,indent=2))
    pd.DataFrame([{'stage':'alt', 'rank':i+1, 'score':v, 'gains':k.tolist()} for i,(v,k) in enumerate(trials)]+[
        {'stage':'roll','rank':i+1,'score':v,'gains':k.tolist()} for i,(v,k) in enumerate(roll_trials)]).to_csv(ROOT/'conventional_training_grid.csv',index=False)
    print('Conventional selected',json.dumps(out),flush=True)
    return out

def tune_auto(kind):
    start=time.time()
    # all 10 seeds, a declared modest computational budget each; never choose based on GATE
    bounds=[(10.,32.),(.1,16.),(4.,16.),(.7,3.2),(0.,.25),(.16,.70)]
    log=[]
    for seed in range(121,131) if kind=='performance' else range(221,231):
        result=differential_evolution(lambda x:pid_obj(x,kind),bounds,seed=seed, maxiter=6,
                                      popsize=3,mutation=.7,recombination=.85,
                                      tol=0.005,polish=False,updating='immediate',workers=1)
        log.append({'seed':seed,'J':float(result.fun),'gains':result.x.tolist(),'nfev':int(result.nfev)})
        print(f'{kind} seed {seed}: J={result.fun:.4f} nfev={result.nfev} elapsed={time.time()-start:.1f}s',flush=True)
    # median-ranked run, predeclared to avoid cherry-picking a fortunate seed.
    log.sort(key=lambda d:(d['J'],d['seed']))
    chosen=log[(len(log)-1)//2]
    # Local bounded refinement only for Performance; robust uses median-rank directly.
    best=chosen['gains'];score=chosen['J'];nm_used=False
    if kind=='performance':
        opt=minimize(lambda g:pid_obj(g,kind),np.array(best),method='Nelder-Mead',
                     options={'maxfev':100,'adaptive':True,'xatol':.005,'fatol':.001})
        x=np.array(opt.x)
        if opt.fun<score and np.all(x>=[b[0] for b in bounds]) and np.all(x<=[b[1] for b in bounds]):
            best=x.tolist();score=float(opt.fun);nm_used=True
    summary={'kind':kind,'gains':best,'J':score,'selected_seed':chosen['seed'],'median_seed_J':chosen['J'],
             'best_DE_J':log[0]['J'],'worst_DE_J':log[-1]['J'],'NM_accepted':nm_used,
             'seeds':[r['seed'] for r in log], 'elapsed_s':time.time()-start}
    pd.DataFrame(log).to_csv(ROOT/f'{kind}_10seed.csv',index=False)
    (ROOT/f'{kind}_selected.json').write_text(json.dumps(summary,indent=2))
    print(kind,'selected',json.dumps(summary),flush=True)
    return summary

def report():
    pc=json.loads((ROOT/'conventional_selected.json').read_text())
    lq=json.loads((ROOT/'lqr_selected.json').read_text())
    ap=json.loads((ROOT/'performance_selected.json').read_text())
    ar=json.loads((ROOT/'robust_selected.json').read_text())
    specs=[('Carefully tuned PID',3,pc['gains'][:3],pc['gains'][3:]),
           ('Performance Auto-PID',3,ap['gains'][:3],ap['gains'][3:]),
           ('Robust Auto-PID',3,ar['gains'][:3],ar['gains'][3:]),
           ('Actuator-aware Integral LQR',2,lq['Kz'],lq['Kr'])]
    rows=[]
    for name,mode,kz,kr in specs:
        for phase in ['train','gate']:
            for r in run_all(mode,kz,kr,phase):rows.append(dict(controller=name,phase=phase,**r))
    df=pd.DataFrame(rows);df.to_csv(ROOT/'paired_development_and_gate_trials.csv',index=False)
    agg=df.groupby(['phase','controller','delay_ms'],as_index=False).agg(
        n=('passes','count'),passes=('passes','sum'),mean_settle_s=('settling_s','mean'),
        mean_IAE=('IAE','mean'),mean_roll_IAE_deg_s=('roll_IAE_deg_s','mean'),
        mean_OS_pct=('overshoot_pct','mean'),mean_saturation_pct=('saturation_pct','mean'),
        mean_uRMS_N=('uRMS_N','mean'),max_xy_drift_m=('max_xy_drift_m','max'))
    agg['pass_percent']=100*agg['passes']/agg['n']
    agg.to_csv(ROOT/'comparison_summary.csv',index=False)
    print('Comparison summary:\n',agg.to_string(index=False,float_format=lambda a:f'{a:.3f}'),flush=True)
    return agg

def write_protocol():
    meta={'status':'DEVELOPMENT ONLY: controllers are provisional; NOT FINAL FROZEN',
      'source':'reconstructed implementation from V1 manuscript and working protocol; prior allocator audit',
      'allocator':'collective priority: fixed f0=f2=T/4, feasible roll-torque clipping; no pitch/yaw commands',
      'plant':{'dt':m.DT,'motor_tau':m.TAU_M,'nominal_mass':m.M_NOM,'arm_m':m.ARM,'max_thrust_per_rotor_N':12.,
         'model_reconstruction_assumptions':['linear_drag=0.12','angular_damping=0.015','derivative_filter_tau=.04','PID kaw=.5','km/kf=.02']},
      'measurement':{'z_std_m':NOISE_SIGMA_Z,'roll_std_deg':NOISE_SIGMA_ROLL_DEG,'train_noise_seeds':TUNE_SEEDS,
          'train_delays_ms':[10*d for d in TUNE_DELAYS], 'gate_noise_seeds':GATE_SEEDS,'gate_delays_ms':[10*d for d in GATE_DELAYS]},
      'training_scenarios':[{'name':n,'plant':list(p),'uncertainty_scales':list(u)} for n,p,u in TRAIN],
      'gate_scenarios':[{'name':n,'plant':list(p),'uncertainty_scales':list(u)} for n,p,u in GATE],
      'primary_metrics':['stability','altitude settling time','altitude overshoot','altitude IAE','roll IAE','control effort uRMS','rotor saturation','final altitude error'],
      'known_challenges':['S1','S2','S3'],'known_challenge_used_to_tune':False,
      'final_100_condition_set':'NOT GENERATED OR INSPECTED',
      'limitations':['training cases are not independent','gate is diagnostic not final held-out','noise Gaussian IID and delay quantization assumed, not experimental sensors','altitude and roll loops only; uncontrolled x-y/yaw','method uses same measurement observations but not necessarily same CPU tuning budget','LQR rate filtering and motor predictor are nominal model-based']}
    (ROOT/'protocol.json').write_text(json.dumps(meta,indent=2))

if __name__=='__main__':
    write_protocol(); import argparse
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['conventional','lqr','performance','robust','report']);v=p.parse_args()
    if v.stage=='conventional':tune_pid_conventional()
    elif v.stage=='lqr':tune_lqr()
    elif v.stage in ['performance','robust']:tune_auto(v.stage)
    elif v.stage=='report':report()
