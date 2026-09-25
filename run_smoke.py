"""Smoke tests: full UAL + AMR machinery on mini cases."""
import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from asq import benchmarks as B
from asq.model import Problem
from asq.kernels import VAR_SC, VAR_CGD, VAR_MNLD


def smoke(name, case, tmax=600):
    print(f"--- {name} ---")
    t0 = time.time()
    p = Problem(case)
    print(f"  init mesh: nelem={p.mesh.nelem} nnode={p.mesh.nnode} "
          f"dof={3 * p.mesh.nnode}")
    try:
        rec = p.run(verbose=True)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print(f"  CRASHED: {exc}")
        return None
    u = np.array(rec['u'])
    F = np.array(rec['F'])
    print(f"  steps={len(u)} u_end={u[-1]:.5g} Fpeak={F.max():.5g} "
          f"@u={u[F.argmax()]:.5g} amr={p.amr_count} t={time.time()-t0:.1f}s")
    # sanity: curve should rise then fall
    ok = F.max() > 0 and len(u) > 5
    print(f"  [{'OK' if ok else 'FAIL'}]")
    return rec


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if which in ('all', 'mini_sns'):
        smoke('mini_sns sc', B.mini_sns(VAR_SC))
    if which in ('all', 'mini_sns_pf'):
        smoke('mini_sns cgd', B.mini_sns(VAR_CGD))
    if which in ('all', 'mini_pf'):
        smoke('mini_snt_pf', B.mini_snt_pf())
