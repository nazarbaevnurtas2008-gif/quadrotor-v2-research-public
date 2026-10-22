"""V2 actuator-aware discrete-time Integral-LQR development-only experiment.
Reconstructed from N. Nazarbayev's manuscript and Quadrotor V2 Working Code & Protocol.
NOT original/recovered code. This script does NOT generate or open the final held-out set.

Conditions and additional assumptions are listed in README.md.
"""
import json
from itertools import product
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.linalg import expm, solve_discrete_are
from numba import njit

ROOT = Path(__file__).resolve().parent
DT = 0.01
G = 9.81
M_NOM = 1.5
IXX = .034
IYY = .034
IZZ = .060
ARM = .23
TAU_M = .045
OMEGA_MAX = 950.
KF = 12./OMEGA_MAX**2
DRAG = .12 # undocumented/reconstruction assumption
DAMP = .015 # undocumented/reconstruction assumption
FILTER_TAU = .04 # undocumented/reconstruction assumption
KAW = .5 # undocumented/reconstruction assumption

# Development set constructed without final set, deliberately excludes 3.0 kg S3.
# columns mass, initial roll deg, vertical gust m/s, x wind m/s, roll torque Nm, gust start,end
DEV_CASES = np.array([
    [1.5,5.,0.,0.,0.,4.,5.],
    [1.7,8.,-0.8,3.,0.025,4.,5.],
    [2.2,5.,0.,0.,0.,4.,5.],
    [1.4,7.,-1.0,4.,-0.020,4.,5.],
],dtype=float)
# Only read after selection is already complete, used for illustrative post-selection checks.
REQUIRED = {
  'S1': [1.5,8.,0.,0.,0.,4.,5.],
  'S2': [1.5,8.,-2.,6.,0.06,4.,5.],
  'S3': [3.,8.,0.,0.,0.,4.,5.],
}

@njit(cache=True)
def dynamics(x, omega_cmd, mass, wx, wz, ext_roll, mot_sc, drag_sc, in_sc, thrust_sc):
    deriv=np.zeros(17)
    deriv[0:3]=x[3:6]
    w,xq,yq,zq=x[6:10]
    p,q,r=x[10:13]
    Om=x[13:17]
    f=KF*thrust_sc*Om*Om
    T=f[0]+f[1]+f[2]+f[3]
    tx=ARM*(f[1]-f[3])
    ty=ARM*(f[2]-f[0])
    tz=.020*(f[0]-f[1]+f[2]-f[3])
    # third column of body -> world rotation, derived from quaternion
    bx=2*(xq*zq+w*yq)
    by=2*(yq*zq-w*xq)
    bz=1.-2*(xq*xq+yq*yq)
    deriv[3]=(T*bx - DRAG*drag_sc*(x[3]-wx))/mass
    deriv[4]=(T*by - DRAG*drag_sc*x[4])/mass
    deriv[5]=(T*bz - DRAG*drag_sc*(x[5]-wz))/mass - G
    deriv[6]=-.5*(xq*p+yq*q+zq*r)
    deriv[7]= .5*(w*p+yq*r-zq*q)
    deriv[8]= .5*(w*q+zq*p-xq*r)
    deriv[9]= .5*(w*r+xq*q-yq*p)
    deriv[10]=(tx+ext_roll - (IZZ-IYY)*in_sc*q*r - DAMP*p)/(IXX*in_sc)
    deriv[11]=(ty - (IXX-IZZ)*in_sc*p*r - DAMP*q)/(IYY*in_sc)
    deriv[12]=(tz - (IYY-IXX)*in_sc*p*q - DAMP*r)/(IZZ*in_sc)
    for j in range(4):
        deriv[13+j]=(omega_cmd[j]-Om[j])/(TAU_M*mot_sc)
    return deriv

@njit(cache=True)
def step_rk4(x, omega_cmd, mass, wx, wz, ext_roll, mot_sc, drag_sc, in_sc, thrust_sc):
    k1=dynamics(x,omega_cmd,mass,wx,wz,ext_roll,mot_sc,drag_sc,in_sc,thrust_sc)
    k2=dynamics(x+.5*DT*k1,omega_cmd,mass,wx,wz,ext_roll,mot_sc,drag_sc,in_sc,thrust_sc)
    k3=dynamics(x+.5*DT*k2,omega_cmd,mass,wx,wz,ext_roll,mot_sc,drag_sc,in_sc,thrust_sc)
    k4=dynamics(x+DT*k3,omega_cmd,mass,wx,wz,ext_roll,mot_sc,drag_sc,in_sc,thrust_sc)
    xn=x+DT*(k1+2*k2+2*k3+k4)/6.
    n=np.sqrt(np.sum(xn[6:10]**2))
    xn[6:10]/=n
    return xn

@njit(cache=True)
def roll_from_quat(q):
    return np.arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1.-2*(q[1]*q[1]+q[2]*q[2]))

@njit(cache=True)
def allocate(Tcmd, txcmd):
    # Exact unconstrained + mixer for yaw/pitch=0; then independent rotor clipping.
    Tcmd=max(0.,min(48.,Tcmd))
    f=np.array([.25*Tcmd,.25*Tcmd+txcmd/(2*ARM),.25*Tcmd,.25*Tcmd-txcmd/(2*ARM)])
    f=np.clip(f,0.,12.)
    oc=np.sqrt(f/KF)
    return oc,f

@njit(cache=True)
def run_sim(case, kz, kr, mode, nsteps=1201, mot_sc=1., drag_sc=1., in_sc=1., thrust_sc=1., zref=2.):
    """mode 0=legacy fixed PID, 1=tuned PID, 2=4-state LQI, 3=variable-gain PID.
    LQI actuator state uses nominal command-based model prediction, NOT true rotor states.
    No sensor noise/delay in this development-only deterministic experiment.
    """
    mass, roll0, wz, wx, moment, t_start, t_end=case
    x=np.zeros(17)
    x[6]=np.cos(np.deg2rad(roll0)/2.)
    x[7]=np.sin(np.deg2rad(roll0)/2.)
    x[13:17]=np.sqrt(M_NOM*G/(4*KF))
    zarr=np.zeros(nsteps)
    rarr=np.zeros(nsteps)
    dthrust=np.zeros(nsteps)
    saturation=np.zeros(nsteps)
    iz=0.;ir=0.;zrate_hat=0.;rrate_hat=0.
    last_z=0.;last_roll=roll_from_quat(x[6:10])
    cmdT=M_NOM*G; cmdR=0.
    act_hat_T=0.; act_hat_roll=0.
    integral_z=0.;integral_roll=0.
    alt_pid=(14.,3.,6.)
    roll_pid=(.9,.1,.26)
    if mode==1:
        alt_pid=(24.,9.,8.5)
        roll_pid=(1.7,0.,.36)
    elif mode==3:
        alt_pid=(kz[0],kz[1],kz[2])
        roll_pid=(kr[0],kr[1],kr[2])
    for k in range(nsteps):
        t=k*DT
        z=x[2]
        roll=roll_from_quat(x[6:10])
        zarr[k]=z
        rarr[k]=roll
        alpha=DT/(FILTER_TAU+DT)
        zrate_hat+=alpha*((z-last_z)/DT-zrate_hat)
        rrate_hat+=alpha*((roll-last_roll)/DT-rrate_hat)
        last_z=z;last_roll=roll
        # shared observation, both use filtered finite-difference estimates
        errz=zref-z
        errr=-roll
        if mode==2:
            dz_cmd=-(kz[0]*(-errz) + kz[1]*zrate_hat + kz[2]*act_hat_T + kz[3]*iz)
            dr_cmd=-(kr[0]*roll + kr[1]*rrate_hat + kr[2]*act_hat_roll + kr[3]*ir)
            rawT=M_NOM*G+dz_cmd
            rawR=dr_cmd
            cmdT=max(0.,min(48.,rawT))
            cmdR=max(-4.,min(4.,rawR))
            # Conditional anti-windup for LQI; integration uses measured error.
            if not ((rawT>48. and errz>0.) or (rawT<0. and errz<0.)):
                iz+=DT*(-errz)
            if not ((rawR>4. and errr>0.) or (rawR< -4. and errr<0.)):
                ir+=DT*(-errr)
            iz=max(-10.,min(10.,iz))
            ir=max(-2.,min(2.,ir))
            # nominal actuator predictor driven by saturated command, not plant omega
            act_hat_T+=DT*((cmdT-M_NOM*G)-act_hat_T)/TAU_M
            act_hat_roll+=DT*(cmdR-act_hat_roll)/TAU_M
        else:
            kp,ki,kd=alt_pid
            krp,kri,krd=roll_pid
            unsat_z=kp*errz+ki*integral_z-kd*zrate_hat
            sat_z=max(-M_NOM*G,min(48.-M_NOM*G,unsat_z))
            unsat_r=krp*errr+kri*integral_roll-krd*rrate_hat
            sat_r=max(-4.,min(4.,unsat_r))
            integral_z=max(-20.,min(20.,integral_z+DT*(errz+KAW*(sat_z-unsat_z))))
            integral_roll=max(-2.,min(2.,integral_roll+DT*(errr+KAW*(sat_r-unsat_r))))
            cmdT=M_NOM*G+sat_z
            cmdR=sat_r
        omega_cmd,f=allocate(cmdT,cmdR)
        dthrust[k]=cmdT-M_NOM*G
        saturation[k]= 1. if (np.min(f)<1e-6 or np.max(f)>=12.-1e-6) else 0.
        wind_on=(t>=t_start and t<=t_end)
        x=step_rk4(x, omega_cmd, mass, wx if wind_on else 0., wz if wind_on else 0., moment if wind_on else 0.,mot_sc,drag_sc,in_sc,thrust_sc)
        if not np.all(np.isfinite(x)) or np.max(np.abs(x))>1.e5:
            zarr[k:]=np.nan
            rarr[k:]=np.nan
            break
    return zarr,rarr,dthrust,saturation


def metrics(z,roll,u,sat,zref=2.):
    if not (np.all(np.isfinite(z)) and np.all(np.isfinite(roll))):
        return dict(stable=False,settling_s=12.,overshoot_pct=np.nan,IAE=np.nan,roll_IAE_deg_s=np.nan,final_error_m=np.nan,saturation_pct=np.nan,uRMS_N=np.nan,passes=False)
    outside=np.flatnonzero(np.abs(z-zref)>max(.05*zref,1.e-6))
    settle=min(12., (int(outside[-1])+1)*DT) if len(outside) else 0.
    os=max(0.,(np.max(z)-zref)/zref*100.)
    iae=np.trapezoid(np.abs(z-zref),dx=DT)
    r_iae=np.trapezoid(np.abs(roll),dx=DT)*180./np.pi
    err=zref-z[-1]
    stable=np.max(np.abs(roll))<np.deg2rad(65.) and np.max(np.abs(z))<25.
    satpc=np.mean(sat)*100.
    urms=np.sqrt(np.mean(u*u))
    return dict(stable=bool(stable),settling_s=float(settle),overshoot_pct=float(os),IAE=float(iae),roll_IAE_deg_s=float(r_iae),final_error_m=float(err),saturation_pct=float(satpc),uRMS_N=float(urms),passes=bool(stable and settle<=5.8 and os<=14. and abs(err)<=.085))


def lqi_gain(channel, weights, r):
    if channel=='alt':
        inertia=M_NOM; drag=DRAG
    else:
        inertia=IXX; drag=DAMP
    A=np.zeros((4,4))
    A[0,1]=1.
    A[1,1]=-drag/inertia
    A[1,2]=1./inertia
    A[2,2]=-1./TAU_M
    A[3,0]=1.
    B=np.array([[0.],[0.],[1./TAU_M],[0.]])
    Z=np.zeros((5,5))
    Z[:4,:4]=A
    Z[:4,4:5]=B
    Md=expm(Z*DT)
    Ad=Md[:4,:4];Bd=Md[:4,4:5]
    P=solve_discrete_are(Ad,Bd,np.diag(weights),np.array([[r]]))
    K=np.linalg.solve(np.array([[r]])+Bd.T@P@Bd,Bd.T@P@Ad).ravel()
    return K


def calculate_dev(kz,kr):
    rows=[]
    for j,c in enumerate(DEV_CASES,1):
        z,r,u,sat=run_sim(c,kz,kr,2)
        rows.append({'scenario':f'D{j}',**metrics(z,r,u,sat)})
    return rows


def score(rows,phase='alt'):
    # Fixed development-only criteria, including penalties and a control-effort term.
    if not all(row['stable'] for row in rows):return 1e6
    values=[]
    for row in rows:
        if not np.isfinite(row['IAE']):return 1e6
        penalty=15.*max(0.,abs(row['final_error_m'])-.08) + 0.7*max(0.,row['overshoot_pct']-14.)
        penalty+=.7*max(0.,row['saturation_pct']-10.) + .75*max(0.,row['settling_s']-5.8)
        # roll IAE [deg*s] is scored separately to avoid conflating units and priorities
        if phase=='alt':
            values.append(.60*row['IAE']+.25*row['settling_s']+.025*row['overshoot_pct']+.025*row['saturation_pct']+.003*row['uRMS_N'] + penalty)
        else:
            values.append(row['roll_IAE_deg_s']+.08*row['saturation_pct']+.015*row['uRMS_N']+penalty)
    return float(np.mean(values))


def main():
    # Warm up numba & baseline verification: true simulation, no fake output.
    baseline=[]
    for label,mode in [('Legacy Fixed PID',0),('Carefully Tuned PID',1)]:
        for name,c in REQUIRED.items():
            z,r,u,sat=run_sim(np.array(c),np.zeros(4),np.zeros(4),mode)
            baseline.append({'controller':label,'scenario':name,**metrics(z,r,u,sat)})
    pd.DataFrame(baseline).to_csv(ROOT/'baseline_reproduction.csv',index=False)

    # Compact predeclared development grid. Search by stages to reduce eval burden.
    # Altitude Qz: [z_err, vz_hat, deltaT_hat, integral_err]
    # Roll Qr:     [phi_err,p_hat, tau_hat, integral_err]
    alt_q=list(product([15.,35.,65.],[8.,20.],[0.1,.7],[4.,18.]))
    alt_r=[.5,2.]
    roll_q=list(product([20.,45.],[5.,15.],[0.05,.35],[1.,8.]))
    roll_r=[2.,10.]

    # Use preliminary, *fixed* roll LQI chosen from a physically conservative design.
    initial_roll_K=lqi_gain('roll',[30.,8.,.2,3.],8.)
    alt_results=[]
    for qs in alt_q:
        for R in alt_r:
            try:
                K=lqi_gain('alt',qs,R)
                rows=calculate_dev(K,initial_roll_K)
                J=score(rows,'alt')
                alt_results.append({'Q':str(qs),'R':R,'K':str(K.tolist()),'score':J,'all_development_pass':all(d['passes'] for d in rows),'max_OS':max(d['overshoot_pct'] for d in rows),'max_sat_pct':max(d['saturation_pct'] for d in rows),'Kvec':K})
            except (ValueError,np.linalg.LinAlgError):pass
    alt_results.sort(key=lambda x:x['score'])
    # Alt stage is frozen BEFORE roll search, doesn't inspect required/final data
    selected_kz=alt_results[0]['Kvec']
    roll_results=[]
    for qs in roll_q:
        for R in roll_r:
            try:
                K=lqi_gain('roll',qs,R)
                rows=calculate_dev(selected_kz,K)
                roll_results.append({'Q':str(qs),'R':R,'K':str(K.tolist()),'score':score(rows,'roll'),'all_development_pass':all(d['passes'] for d in rows),'max_sat_pct':max(d['saturation_pct'] for d in rows),'Kvec':K})
            except (ValueError,np.linalg.LinAlgError):pass
    roll_results.sort(key=lambda x:x['score'])
    selected_kr=roll_results[0]['Kvec']

    pd.DataFrame([{k:v for k,v in a.items() if k!='Kvec'} for a in alt_results]).to_csv(ROOT/'altitude_lqi_development_grid.csv',index=False)
    pd.DataFrame([{k:v for k,v in a.items() if k!='Kvec'} for a in roll_results]).to_csv(ROOT/'roll_lqi_development_grid.csv',index=False)
    # selected development-only controller; no final data touched
    dev_rows=[]; traces={}
    for label,mode,K1,K2 in [('Carefully Tuned PID',1,np.zeros(4),np.zeros(4)),('4-state Integral LQR (development candidate)',2,selected_kz,selected_kr)]:
        for j,c in enumerate(DEV_CASES,1):
            z,r,u,sat=run_sim(c,K1,K2,mode)
            dev_rows.append({'controller':label,'scenario':f'D{j}',**metrics(z,r,u,sat)})
            if j in [1,3]: traces[(label,f'D{j}')]=(z,r,u)
    pd.DataFrame(dev_rows).to_csv(ROOT/'development_comparison.csv',index=False)
    # Once development selection is fixed, required test is a secondary diagnostic, not tuning input.
    req=[]
    for name,c in REQUIRED.items():
        z,r,u,sat=run_sim(np.array(c),selected_kz,selected_kr,2)
        req.append({'controller':'4-state Integral LQR (development candidate)','scenario':name,**metrics(z,r,u,sat)})
        if name in ['S1','S3']:traces[('LQR',name)]=(z,r,u)
    pd.DataFrame(req).to_csv(ROOT/'required_postselection_check.csv',index=False)
    t=np.arange(1201)*DT
    np.savez_compressed(ROOT/'development_trajectories.npz',time_s=t,**{str(k[0]).replace(' ','_')+'_'+k[1]+'_'+label: val for k,(z,r,u) in traces.items() for label,val in [('altitude_m',z),('roll_rad',r),('deltaT_N',u)]})
    import matplotlib.pyplot as plt
    fig, ax=plt.subplots(figsize=(9,5))
    for (name,cond),(z,r,u) in traces.items():
        if cond in ['D1','D3']:
            ax.plot(t,z,label=name+': '+cond)
    ax.plot(t,np.full_like(t,2.),'--',label='2 m reference',lw=1.2)
    ax.set(xlabel='Time (s)',ylabel='Altitude (m)',title='Development-only trajectory comparison — not final held-out')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ROOT/'development_altitude_trajectories.png',dpi=180)
    plt.close(fig)
    report={
        'status':'Development-only candidate; not final freeze',
        'method':'4-state discrete ZOH LQI, nominal hover, measured outputs + filtered derivatives + model-based actuator prediction',
        'selected_Qz':alt_results[0]['Q'],'selected_Rz':alt_results[0]['R'],'selected_Kz':selected_kz.tolist(),
        'selected_Qroll':roll_results[0]['Q'],'selected_Rroll':roll_results[0]['R'],'selected_Kroll':selected_kr.tolist(),
        'grid_candidates':{'altitude':len(alt_results),'roll':len(roll_results)},
        'altitude_dev_score':alt_results[0]['score'],'roll_dev_score':roll_results[0]['score'],
        'no_final_heldout_generated':True,
        'limitations':['Controller based on documented + reconstructed assumptions; not recovered original code','Development cases deterministic with no injected sensor noise or measurement delay','Roll/yaw uncontrolled beyond roll loop; 6-DOF dynamics integrated but only 2 active control channels','Reconstruction lacks source calibration for motor/drag/noise and gains are not safety validated','Actuator estimator uses nominal motor lag; actual rotor dynamics nonlinear']}
    (ROOT/'development_summary.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))
    print('Development comparison:\n',pd.DataFrame(dev_rows).to_string(index=False,float_format=lambda v:f'{v:.3f}'))
    print('Post-selection required checks:\n',pd.DataFrame(req).to_string(index=False,float_format=lambda v:f'{v:.3f}'))

if __name__=='__main__': main()
