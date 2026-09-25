"""Collect results/*_rec.json into the report comparison tables.

Usage: python collect_results.py
Prints markdown tables + writes report/results_summary.json
"""
import sys
import os
import json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_bench import JIN_ANCHORS

R = 'results'
OUT = {}


def peak_of(u, F):
    j = int(np.argmax(F))
    return float(u[j]), float(F[j])


def curve_at(u, F, target):
    """piecewise-linear interpolation of F at u=target."""
    u = np.asarray(u)
    F = np.asarray(F)
    order = np.argsort(u)
    u, F = u[order], F[order]
    # deduplicate
    keep = np.concatenate(([True], np.diff(u) > 0))
    u, F = u[keep], F[keep]
    if len(u) < 2:
        return None
    return float(np.interp(target, u, F))


rows_pf = []
rows_gd = []
for name in sorted(os.listdir(R)):
    if not name.endswith('_rec.json'):
        continue
    case = name[:-9]
    try:
        r = json.load(open(os.path.join(R, name)))
    except Exception:
        continue
    u = np.array(r['u'])
    F = np.array(r['F'])
    up, Fp = peak_of(u, F)
    entry = dict(
        case=case,
        u_peak=up, F_peak=Fp,
        u_end=float(u[-1]), F_end=float(F[-1]),
        steps=len(u), newton=int(r.get('nnewton', 0)),
        amr=int(r.get('amr_count', 0)),
        dof0=int(r['dof'][0]), dof_end=int(r['dof'][-1]),
        walltime=float(r.get('walltime', 0.0)),
    )
    if 'pf' in case:
        # compare against digitised anchors
        a = JIN_ANCHORS.get(case.replace('_pe', '').replace('_ps', ''))
        if a:
            i = int(np.argmax(a['F']))
            entry['ref_u_peak'] = a['u'][i]
            entry['ref_F_peak'] = a['F'][i]
            entry['F_at_ref_points'] = {
                f"F@u={ut}": curve_at(u, F, ut)
                for ut in (0.0096, 0.012, 0.015, 0.0059, 0.006)
                if u[-1] >= ut}
    OUT[case] = entry
    row = [case, f"{Fp:.4f}", f"{up:.5f}", f"{len(u)}",
           f"{r.get('amr_count', 0)}", f"{r['dof'][-1]}",
           f"{r.get('walltime', 0):.0f}s"]
    (rows_pf if '_pf' in case else rows_gd).append(row)

print("## PF validation (vs Jin 2024 digitised)\n")
print("| case | F_peak [kN] | u_peak | steps | AMR | dof_end | wall |")
print("|---|---|---|---|---|---|---|")
for row in rows_pf:
    print("| " + " | ".join(row) + " |")
print("\n## GD comparison\n")
print("| case | F_peak [kN] | u_peak | steps | AMR | dof_end | wall |")
print("|---|---|---|---|---|---|---|")
for row in rows_gd:
    print("| " + " | ".join(row) + " |")

with open('report/results_summary.json', 'w') as f:
    json.dump(OUT, f, indent=1)
print("\n[written] report/results_summary.json")
