#!/usr/bin/env python3
"""Analytic and frozen-model hover consistency; NOT experimental flight validation."""
import sys,json,numpy as np
from pathlib import Path
BASE=Path(__file__).resolve().parent
ROOT=BASE/'frozen_evidence'
sys.path.insert(0,str(ROOT/'frozen_source'/'reference_model'))
import v2_lqr_development as model
m_nom=model.M_NOM; g=model.G
omega=np.sqrt(m_nom*g/(4*model.KF))
x=np.zeros(17);x[6]=1.;x[13:17]=omega
xdot=model.dynamics(x,np.full(4,omega),m_nom,0.,0.,0.,1.,1.,1.,1.)
check={'hover_nominal_mass_kg':float(m_nom),'hover_collective_N':float(m_nom*g),
'per_rotor_N':float(m_nom*g/4),'omega_hover_rad_s':float(omega),'nominal_max_omega_rad_s':float(model.OMEGA_MAX),
'hover_derivative_inf_norm':float(np.max(np.abs(xdot))),
'model_rotor_thrust_cap_at_scale_1':float(model.KF*model.OMEGA_MAX**2),
'model_rotor_thrust_cap_at_scale_1_08':float(1.08*model.KF*model.OMEGA_MAX**2),
'nominal_hover_equilibrium':bool(np.max(np.abs(xdot))<1e-12)}
(BASE/'physical_hover_invariants.json').write_text(json.dumps(check,indent=2))
print(json.dumps(check,indent=2))
