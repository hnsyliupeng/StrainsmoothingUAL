"""Debug the spurious total-collapse at jin_snt_pf handover (u=0.00236)."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asq import benchmarks as B
from asq.model import Problem

case = B.jin_snt_pf()
p = Problem(case)
n = p.mesh.nnode
lam = 0.0
dlam = case['dlam0']
step = 0
q_prev = 0.0
while lam < case['lam_end']:
    step += 1
    target = min(lam + dlam, case['lam_end'])
    Uc, lamc, ok = p.nr_fixed(target, U0=p.U, lam0=lam, tol=1e-9)
    if not ok:
        dlam *= 0.25
        step -= 1
        continue
    p.U, lam = Uc, lamc
    p.lam = lam
    q = p._onset_ratio()
    dq = q - q_prev
    q_prev = q
    dmax = float(p.d_sub.max())
    print(f"step {step}: lam={lam:.6f} dmax={dmax:.4f} q={q:.3f} dlam={dlam:.2e}")
    if dmax > case['onset_d']:
        break
    if q < 0.5:
        dlam = min(dlam * 2.0, 0.05 * case['lam_end'])
    elif dq > 0.2:
        dlam *= 0.2 / max(dq, 0.02)

print("\n=== handover state reached; now probe fallback step ===")
U0 = p.U.copy()
lam0 = lam
print(f"state: lam={lam0:.6f} dmax={p.d_sub.max():.4f} Hmax={p.H.max():.4g}"
      f" psi_c={p.psi_c:.4g} nsub={p.sf['nsub']}")

# instrumented nr_fixed
orig_assemble = p.assemble

def probe(tag, target):
    U = U0.copy()
    p.U = U0.copy()
    p.lam = lam0
    free = p.free
    Dy = np.zeros(p.nfree)
    print(f"\n--- {tag}: target lam={target:.6f} ---")
    for it in range(30):
        lam = lam0 + (target - lam0) * 1.0
        p.U = U
        p.lam = target
        Up = p.set_U_prescribed(p.U.copy(), target)
        Uf = Up
        rf, Jff, qv = p._free_system(Uf)
        rn = float(np.linalg.norm(rf))
        dmax = float(p.d_sub.max()) if p.d_sub is not None else -1
        nd9 = int((p.d_sub > 0.9).sum()) if p.d_sub is not None else -1
        nd5 = int((p.d_sub > 0.5).sum()) if p.d_sub is not None else -1
        print(f"  it{it:2d}: rn={rn:.3e} dmax={dmax:.4f} d>0.5:{nd5:5d} "
              f"Hmax={p.H.max():.3e}")
        if rn < 1e-9:
            print("  converged (machine)")
            return True
        from scipy.sparse.linalg import splu
        lu = splu(Jff)
        dy = lu.solve(-rf)
        # line search
        best = None
        for eta in (1.0, 0.5, 0.25, 0.125, 0.0625):
            Ut = U0.copy()
            Ut[free] += Dy + eta * dy
            Ut = p.set_U_prescribed(Ut, target)
            p.U = Ut
            rn_t = p.residual_norm(Ut)
            if rn_t < rn:
                best = (eta, Ut, Dy + eta * dy, rn_t)
                break
        if best is None:
            print(f"  line search failed (rn={rn:.3e})")
            return False
        eta, U, Dy, rn_new = best
        print(f"        eta={eta} rn-> {rn_new:.3e}")
    return False

probe("fallback-size step (dlam=1e-4)", lam0 + 1e-4)
probe("small step (dlam=1e-5)", lam0 + 1e-5)
probe("tiny step (dlam=1e-6)", lam0 + 1e-6)
