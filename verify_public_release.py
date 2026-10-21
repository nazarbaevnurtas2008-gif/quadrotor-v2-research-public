"""Check the completeness of the publicly bundled frozen V2 evaluation CSV.
This is NOT a proof of physical model validity or statistical generalization.
"""
import csv
from collections import defaultdict
from pathlib import Path
r=Path(__file__).resolve().parent
p=r/'frozen_evidence/final_400_paired_controller_trials.csv'
rows=list(csv.DictReader(p.open(newline='',encoding='utf-8')))
expected={'Carefully tuned conventional PID':95,'Performance Auto-PID':95,
          'Robust Auto-PID':93,'Actuator-aware Integral LQR':99}
assert len(rows)==400, f'Expected 400 trials, found {len(rows)}'
by_case=defaultdict(set)
by_controller=defaultdict(list)
for d in rows:
 by_case[d['case_id']].add(d['controller'])
 by_controller[d['controller']].append(d)
assert len(by_case)==100
assert all(set(expected)==c for c in by_case.values())
for name,ns in expected.items():
 d=by_controller[name]
 assert len(d)==100
 passes=sum(row['altitude_pass'].lower()=='true' for row in d)
 assert passes==ns, (name,passes,ns)
print('PASS: frozen V2 CSV has 100 paired cases x 4 controllers; expected pass rates match.')
print('NOTE: this structural CSV check is not an independent physical validation.')
