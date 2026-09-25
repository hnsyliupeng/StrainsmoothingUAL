"""Probe the jin_snt_pf (plane stress) post-peak stall state."""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asq import benchmarks as B
from asq.model import Problem
from scipy.sparse.linalg import splu

case = B.jin_snt_pf()
p = Problem(case)

# run phase A + B but stop at the frozen state (maxd > 1.0 and lam turning)
rec = p.run(verbose=False)
print(f"run done: steps={len(rec['u'])} lam={p.lam:.6f} "
      f"F_end={rec['F'][-1]:.4f} maxd={p.d_sub.max():.4f}")

# --- inspect the stall state ---
mask = p._pz_mask()
print(f"\nPZ mask: {mask.sum()} subcells of {len(mask)}")
w = p._pz_weights()
print(f"w.sum = {w.sum():.4e},  nnz = {(w>0).sum()}")
psi = p.psi
psi_c = p.psi_c
print(f"psi max = {psi.max():.4g} (psi_c = {psi_c:.4g}); "
      f"subcells psi>=0.5psi_c: {(psi >= 0.5*psi_c).sum()}")
print(f"d_sub: max={p.d_sub.max():.4f}, >0.95: {(p.d_sub>0.95).sum()}, "
      f">=0.2&<0.95: {((p.d_sub>=0.2)&(p.d_sub<0.95)).sum()}")

# where are the mask subcells?
if mask.any():
    sC = p.sf['sC'][mask]
    print(f"mask centroid range: x [{sC[:,0].min():.3f},{sC[:,0].max():.3f}] "
          f"y [{sC[:,1].min():.3f},{sC[:,1].max():.3f}]")

# --- try a manual UAL step with a small tau ---
U0 = p.U.copy()
lam0 = p.lam
free = p.free
n = p.mesh.nnode
# predictor: tiny generic displacement direction
q_dir = np.zeros(p.nfree)
# use last recorded displacement increment shape: just probe dyB
rf, Jff, qv = p._free_system(p.set_U_prescribed(U0.copy(), lam0))
print(f"\nresidual norm at stall state: {np.linalg.norm(rf):.3e}")
try:
    lu = splu(Jff)
    dyA = lu.solve(-rf)
    dyB = lu.solve(-qv)
    den = float(w @ dyB)
    print(f"w@dyB = {den:.3e}   (degenerate if ~0)")
    print(f"w@dyA = {float(w @ dyA):.3e}")
    # what does the load-direction response look like in the PZ?
    print(f"|dyB| = {np.linalg.norm(dyB):.3e}, |dyA| = {np.linalg.norm(dyA):.3e}")
except Exception as e:
    print("splu failed:", e)

# --- fallback direction probe: forward vs backward displacement steps ---
for dlam in (+8e-5, -8e-5, +4e-5, -4e-5):
    Uc, lamc, ok = p.nr_fixed(lam0 + dlam, U0=U0, lam0=lam0, tol=1e-7,
                              maxit=30)
    F = None
    if ok:
        p.U = Uc
        r, _ = p.assemble(p.U, want_tangent=False)
        F = float(np.abs(r[p.load_dofs]).sum())
    print(f"nr_fixed(lam {lam0+dlam:.6f}) ok={ok} F={F if F else '-'}")
    p.U = U0.copy()
    p.lam = lam0
