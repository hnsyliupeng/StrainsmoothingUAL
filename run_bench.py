"""Run benchmark cases, save curves/plots/fields to results/.

Usage: python run_bench.py <case_name> [...]
"""
import sys
import os
import json
import time
import pickle
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from asq import benchmarks as B
from asq.model import Problem
from asq.kernels import VAR_CGD, VAR_MNLD, VAR_SC

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
os.makedirs(RESULTS, exist_ok=True)

# digitised Jin 2024 anchors (v4; to be replaced by PDF extraction)
JIN_ANCHORS = {
    'jin_snt_pf': dict(F=[0, 0.28, 0.52, 0.72, 0.7565, 0.61, 0.24, 0.02, 0.005],
                       u=[0, 0.0015, 0.003, 0.0045, 0.0059, 0.0062, 0.0065,
                          0.0070, 0.008]),
    'jin_sns_pf': dict(F=[0, 0.20, 0.40, 0.52, 0.5466, 0.50, 0.43, 0.359,
                          0.30, 0.25],
                       u=[0, 0.002, 0.004, 0.0065, 0.0096, 0.011, 0.013,
                          0.015, 0.016, 0.0165]),
    'jin_tpb_pf': dict(F=[0, 0.015, 0.028, 0.036, 0.0379, 0.012, 0.006,
                          0.0047],
                       u=[0, 0.01, 0.02, 0.03, 0.047, 0.055, 0.06, 0.065]),
}

CASES = {
    'jin_snt_pf': lambda: B.jin_snt_pf(),
    'jin_snt_mnld': lambda: B.jin_snt_gd(B.VAR_MNLD),
    'jin_sns_mnld': lambda: B.jin_sns_gd(B.VAR_MNLD),
    'jin_sns_pf': lambda: B.jin_sns_pf(),
    'jin_tpb_pf': lambda: B.jin_tpb_pf(),
    'jin_snt_sc': lambda: B.jin_snt_gd(VAR_SC),
    'jin_snt_cgd': lambda: B.jin_snt_gd(VAR_CGD),
    'jin_sns_sc': lambda: B.jin_sns_gd(VAR_SC),
    'jin_sns_cgd': lambda: B.jin_sns_gd(VAR_CGD),
    'sns100_mnld': lambda: B.sns100(VAR_MNLD),
    'sns100_sc': lambda: B.sns100(VAR_SC),
    'sns100_cgd': lambda: B.sns100(VAR_CGD),
    'tpb2000_mnld': lambda: B.tpb2000(VAR_MNLD),
    'tpb2000_sc': lambda: B.tpb2000(VAR_SC),
    'tpb2000_cgd': lambda: B.tpb2000(VAR_CGD),
    'lshape_sc': lambda: B.lshape(VAR_SC),
    'lshape_mnld': lambda: B.lshape(VAR_MNLD),
}


def field_snapshot(p):
    """nodal damage + ebar/d + principal stress on the current mesh."""
    n = p.mesh.nnode
    xy = p.mesh.xy
    if p.model == 'gd':
        # subcell damage -> nodal average
        dn = np.zeros(n)
        cnt = np.zeros(n)
        ef, ep, en = (p.sf['elem_flat'], p.sf['elem_ptr'], p.sf['elem_n'])
        from asq.kernels import damage_law
        dsub = p.d_sub
        for si in range(p.sf['nsub']):
            e = p.sf['sub_el'][si]
            for v in ef[ep[e]:ep[e] + en[e]]:
                dn[v] += dsub[si]
                cnt[v] += 1
        dn /= np.maximum(cnt, 1)
        ebar = p.U[2 * n:]
        return dict(xy=xy, d=dn, ebar=ebar, sig=None)
    else:
        dn = np.zeros(n)
        cnt = np.zeros(n)
        ef, ep, en = (p.sf['elem_flat'], p.sf['elem_ptr'], p.sf['elem_n'])
        dsub = p.d_sub
        for si in range(p.sf['nsub']):
            e = p.sf['sub_el'][si]
            for v in ef[ep[e]:ep[e] + en[e]]:
                dn[v] += dsub[si]
                cnt[v] += 1
        dn /= np.maximum(cnt, 1)
        return dict(xy=xy, d=dn, ebar=p.U[2 * n:], sig=p.sig.copy())


def run_case(name, case=None):
    if case is None:
        case = CASES[name]()
    print(f"=== {name} ===")
    t0 = time.time()
    p = Problem(case)
    print(f"  init: nelem={p.mesh.nelem} nnode={p.mesh.nnode}")
    rec = p.run(verbose=True)
    print(f"  total {time.time()-t0:.1f}s")
    # save
    out = dict(rec)
    out['case'] = {k: v for k, v in case.items()
                   if k not in ('mask', 'slits')}
    with open(f"{RESULTS}/{name}_rec.json", 'w') as f:
        json.dump({k: (list(map(float, v)) if isinstance(v, list) else v)
                   for k, v in out.items() if k != 'timing'}, f)
    snap = field_snapshot(p)
    with open(f"{RESULTS}/{name}_snap.pkl", 'wb') as f:
        pickle.dump(snap, f)
    # plot
    u = np.array(rec['u'])
    F = np.array(rec['F'])
    fig, ax = plt.subplots(figsize=(5, 4))
    ax.plot(u, F, 'b-', lw=2, label=name)
    if name in JIN_ANCHORS:
        a = JIN_ANCHORS[name]
        ax.plot(a['u'], a['F'], 'ro', ms=4, label='Jin 2024 (digitised)')
    ax.set_xlabel('u [mm]')
    ax.set_ylabel('F [kN]')
    ax.set_title(name)
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{RESULTS}/{name}_curve.png", dpi=130)
    plt.close(fig)
    # damage contour
    fig, ax = plt.subplots(figsize=(6, 4))
    sc = ax.tricontourf(snap['xy'][:, 0], snap['xy'][:, 1], snap['d'],
                        levels=np.linspace(0, 1, 21), cmap='magma')
    fig.colorbar(sc, label='damage')
    ax.set_aspect('equal')
    ax.set_title(f"{name} final damage")
    fig.tight_layout()
    fig.savefig(f"{RESULTS}/{name}_damage.png", dpi=130)
    plt.close(fig)
    # summary vs anchors
    if name in JIN_ANCHORS:
        a = JIN_ANCHORS[name]
        i = int(np.argmax(a['F']))
        print(f"  REF peak {a['F'][i]:.4g} @ u={a['u'][i]:.4g}")
    j = int(np.argmax(F))
    print(f"  OUR peak {F[j]:.4g} @ u={u[j]:.4g}  "
          f"(u_end={u[-1]:.4g}, steps={len(u)})")
    return p, rec


if __name__ == '__main__':
    for name in sys.argv[1:]:
        try:
            run_case(name)
        except Exception:
            import traceback
            traceback.print_exc()
