"""
Unit + verification tests for the asq package.
Run:  python -m tests.test_basic
"""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from asq import quadtree as qt
from asq import kernels
from asq.sfem import build_sfem
from asq.model import Problem, _build_pattern
from asq.kernels import VAR_CGD, VAR_MNLD, VAR_SC

rng = np.random.default_rng(42)
FAIL = []


def check(name, ok, msg=''):
    print(f"  [{'OK' if ok else 'FAIL'}] {name} {msg}")
    if not ok:
        FAIL.append(name)


# ----------------------------------------------------------------------
def test_quadtree():
    print("== quadtree ==")
    # --- 2:1 balance (independent O(n^2) adjacency check) ---
    t = qt.Quadtree(0, 0, 4, 4, 2, 2, max_depth=8)
    t.refine_cell((0, 1, 1))
    t.refine_cell((1, 2, 2))
    t.refine_cell((2, 4, 4))
    t.balance()
    leaves = t.leaves()

    def bbox(k):
        return t.cell_bbox(k)

    ok_bal = True
    for i, A in enumerate(leaves):
        a0, b0, a1, b1 = bbox(A)
        for B in leaves[i + 1:]:
            c0, d0, c1, d1 = bbox(B)
            # shared vertical edge?
            touch = ((abs(a1 - c0) < 1e-12 or abs(c1 - a0) < 1e-12)
                     and min(b1, d1) - max(b0, d0) > 1e-12)
            touch = touch or ((abs(b1 - d0) < 1e-12 or abs(d1 - b0) < 1e-12)
                              and min(a1, c1) - max(a0, c0) > 1e-12)
            if touch and abs(A[0] - B[0]) > 1:
                ok_bal = False
    check("2:1 balance", ok_bal)
    # area conservation
    area = sum((lambda bb: (bb[2] - bb[0]) * (bb[3] - bb[1]))(bbox(k))
               for k in leaves)
    check("area conservation", abs(area - 16.0) < 1e-10)

    # --- slit mesh: duplication, shared tip, connectivity ---
    t2 = qt.Quadtree(0, 0, 4, 4, 2, 2, slits=[qt.Slit((0, 2), (2, 2))],
                     max_depth=8)
    t2.refine_by_size_function(
        qt.feature_size_function([(2, 2)], 0.25, 2.0, 1.0, 0.3))
    m2 = qt.Mesh(t2)
    xy = m2.xy
    # tip (2,2) exactly once
    ntip = np.sum((np.abs(xy[:, 0] - 2) < 1e-9) & (np.abs(xy[:, 1] - 2) < 1e-9))
    check("slit tip shared (1 copy)", ntip == 1, f"ntip={ntip}")
    # interior slit points duplicated
    dup_ok = True
    for x in (0.5, 1.0, 1.5):
        cnt = np.sum((np.abs(xy[:, 0] - x) < 1e-9)
                     & (np.abs(xy[:, 1] - 2) < 1e-9))
        if cnt != 2:
            dup_ok = False
    check("slit interior nodes duplicated", dup_ok)
    # mouth (0,2): boundary + on slit -> duplicated per side
    nm = np.sum((np.abs(xy[:, 0]) < 1e-9) & (np.abs(xy[:, 1] - 2) < 1e-9))
    check("slit mouth duplicated on boundary", nm == 2, f"n={nm}")
    # each duplicated slit copy is used only by elements on its side
    from collections import defaultdict
    use = defaultdict(list)
    for poly in m2.elems:
        cy_e = xy[poly, 1].mean()
        for v in poly:
            use[v].append(cy_e)
    side_ok = True
    for xv in (0.0, 0.5, 1.0, 1.5):
        idx = np.where((np.abs(xy[:, 0] - xv) < 1e-9)
                       & (np.abs(xy[:, 1] - 2) < 1e-9))[0]
        if len(idx) != 2 and xv > 0.0:
            side_ok = False
        for v in idx:
            cys = use[v]
            if min(cys) < 2 - 1e-9 and max(cys) > 2 + 1e-9:
                side_ok = False
    check("slit copies used by one side only", side_ok)
    # hanging nodes present
    counts = {}
    for poly in m2.elems:
        counts[len(poly)] = counts.get(len(poly), 0) + 1
    check("polygon sizes in {4,5,6,7,8}", set(counts) <= set(range(4, 9)),
          str(counts))
    # subcell triangulation areas sum to cell area
    sf2 = build_sfem(m2)
    tot = sf2['sA'].sum()
    dom = 16.0
    check("subcell areas sum to domain", abs(tot - dom) < 1e-9,
          f"{tot:.6f} vs {dom}")

    # --- L-shape mask ---
    def lmask(a, b, c, d):
        return not (c > 250 + 1e-9 and b < 250 - 1e-9)
    t3 = qt.Quadtree(0, 0, 500, 500, 2, 2, mask=lmask, max_depth=4)
    m3 = qt.Mesh(t3)
    inside = np.array([1 if lmask(*t3.cell_bbox(k)) else 0
                       for k in t3.leaves()])
    check("L-shape 3 roots", len(t3.leaves()) == 3)
    a3 = sum((t3.cell_bbox(k)[2] - t3.cell_bbox(k)[0])
             * (t3.cell_bbox(k)[3] - t3.cell_bbox(k)[1]) for k in t3.leaves())
    check("L-shape area", abs(a3 - (500 * 250 + 250 * 250)) < 1e-9)
    return m2, sf2


# ----------------------------------------------------------------------
def mini_case(model='gd', variant=VAR_SC):
    return dict(
        model=model, variant=variant,
        x0=0, y0=0, x1=4, y1=4, nx=2, ny=2, max_depth=6,
        slits=[qt.Slit((0, 2), (2, 2))],
        h_min=0.25, h_max=2.0,
        init_features=[dict(pts=[(2, 2)], grade=1.0, r=0.3)],
        fix=[(('edge', 0, 0, 4, 0), 'y'), (('point', 0, 0), 'x')],
        load=[(('edge', 0, 4, 4, 4), 'y', 1.0)],
        mat=dict(lam=1.0, mu=1.0, lc=0.5, epsD=0.01, alpha=0.7,
                 beta_law=100.0, s1=1.5, s2=5.0, dmax=0.999, n_fr=100.0,
                 ST=1e-7, eq_type=kernels.EQ_MAZARS, law=kernels.LAW_MAZARS,
                 k0=10.0, nu=0.2, kappa_s=1e-6),
        lam_end=0.05, u_scale=1.0, dlam0=0.005, tau0=1e-5, dl0=1e-4)


def test_patch(m2, sf2):
    print("== CS-FEM patch test (mesh with slit + hanging nodes) ==")
    mesh = m2
    sf = sf2
    n = mesh.nnode
    ep, en, ef = sf['elem_ptr'], sf['elem_n'], sf['elem_flat']
    pattern, idx_map, gloc = _build_pattern(mesh, sf)
    vals = np.zeros((mesh.nelem, 24, 24))
    rvals = np.zeros((mesh.nelem, 24))
    sub_off = np.zeros(mesh.nelem + 1, np.int64)
    np.cumsum(en, out=sub_off[1:])
    C = kernels.elastic_C(1.2, 0.8)
    mp = np.zeros(21)
    mp[0], mp[1] = 1.2, 0.8
    mp[2] = 0.5 ** 2 / 2
    mp[3] = 1e30      # no damage
    mp[11] = VAR_CGD
    # linear displacement field
    a0, a1, a2, b0, b1, b2 = 0.01, 0.02, -0.015, 0.005, -0.01, 0.017
    xy = mesh.xy
    U = np.zeros(3 * n)
    U[0:2 * n:2] = a0 + a1 * xy[:, 0] + a2 * xy[:, 1]
    U[1:2 * n:2] = b0 + b1 * xy[:, 0] + b2 * xy[:, 1]
    exx = a1
    eyy = b2
    gxy = a2 + b1
    eq, _, _, _ = kernels.eq_strain(exx, eyy, gxy, kernels.EQ_MAZARS, 10, 0.2)
    U[2 * n:] = eq
    kappa = np.zeros(sf['nsub'])
    d_prev = np.zeros(sf['nsub'])
    H = np.zeros(sf['nsub'])
    kernels.assemble_gd(ep, en, ef, n, sf['sB'], sf['sT'], sf['sG'],
                        sf['sM'], sf['sD'], sf['sA'], sub_off,
                        U, kappa, d_prev, False, mp, C, vals, rvals,
                        kappa, np.zeros(sf['nsub']), np.zeros(sf['nsub']),
                        np.zeros(sf['nsub']), True)
    r = np.bincount(gloc.reshape(-1), weights=rvals.reshape(-1),
                    minlength=3 * n + 1)[:3 * n]
    # interior dofs = nodes not on outer boundary
    onb = ((xy[:, 0] < 1e-9) | (xy[:, 0] > 4 - 1e-9)
           | (xy[:, 1] < 1e-9) | (xy[:, 1] > 4 - 1e-9))
    onb |= (np.abs(xy[:, 1] - 2) < 1e-9) & (xy[:, 0] < 2 + 1e-9)
    interior = np.where(~onb)[0]
    scale = np.abs(r).max()
    rmax = 0.0
    for v in interior:
        rmax = max(rmax, abs(r[2 * v]), abs(r[2 * v + 1]),
                   abs(r[2 * n + v]))
    check("patch: interior residual ~ 0", rmax < 1e-9,
          f"rmax={rmax:.2e} scale={scale:.2e}")
    # stress recovery exact
    ebar_sub = np.zeros(sf['nsub'])
    for e in range(mesh.nelem):
        m = en[e]
        vids = ef[ep[e]:ep[e] + m]
        v = U[2 * n:][vids]
        s0, s1 = sub_off[e], sub_off[e + 1]
        ebar_sub[s0:s1] = sf['sD'][s0:s1, :m] @ v
    check("patch: eps_eq recovered", np.abs(ebar_sub - eq).max() < 1e-12)


# ----------------------------------------------------------------------
def test_tangent_gd(m2, sf2):
    print("== FD tangent check: gradient damage variants ==")
    mesh, sf = m2, sf2
    n = mesh.nnode
    ep, en, ef = sf['elem_ptr'], sf['elem_n'], sf['elem_flat']
    pattern, idx_map, gloc = _build_pattern(mesh, sf)
    vals = np.zeros((mesh.nelem, 24, 24))
    rvals = np.zeros((mesh.nelem, 24))
    sub_off = np.zeros(mesh.nelem + 1, np.int64)
    np.cumsum(en, out=sub_off[1:])
    C = kernels.elastic_C(121.15, 80.77)
    xy = mesh.xy

    def build_U(epsD):
        U = np.zeros(3 * n)
        U[0:2 * n:2] = 0.02 * xy[:, 0] + 0.004 * np.sin(3 * xy[:, 1])
        U[1:2 * n:2] = 0.01 * xy[:, 1] + 0.004 * np.cos(3 * xy[:, 0])
        U[2 * n:] = epsD * (1.0 + 0.8 * np.sin(2 * xy[:, 0])
                            * np.cos(2 * xy[:, 1]))
        return U

    for variant, name in ((VAR_CGD, 'cgd'), (VAR_MNLD, 'mnld'),
                          (VAR_SC, 'sc')):
        for eqt, eqn in ((kernels.EQ_MAZARS, 'mazars'),
                         (kernels.EQ_VM, 'vm')):
            epsD = 0.02
            mp = np.zeros(21)
            mp[0], mp[1] = 121.15, 80.77
            mp[2] = 0.25
            mp[3] = epsD
            mp[4] = 0.7
            mp[5] = 150.0
            mp[6], mp[7] = 1.5, 5.0
            mp[8] = 0.999
            mp[9] = 100.0
            mp[10] = 1e-7
            mp[11] = variant
            mp[12] = eqt
            mp[13] = kernels.LAW_MAZARS if eqn == 'mazars' else kernels.LAW_GEERS
            mp[14], mp[15] = 10.0, 0.2
            mp[16:21] = (0.8, 1.0, 2.0, 1000.0, 1e-6)
            U = build_U(epsD)
            ns = sf['nsub']
            eb_sub = np.zeros(ns)
            for e in range(mesh.nelem):
                m = en[e]
                vids = ef[ep[e]:ep[e] + m]
                v = U[2 * n:][vids]
                s0, s1 = sub_off[e], sub_off[e + 1]
                eb_sub[s0:s1] = sf['sD'][s0:s1, :m] @ v
            kp = eb_sub + (rng.random(ns) - 0.5) * 0.02 * epsD
            dp = 0.4 * rng.random(ns)
            kout = np.zeros(ns)
            dout = np.zeros(ns)
            yout = np.zeros(ns)
            pout = np.zeros(ns)
            kernels.assemble_gd(ep, en, ef, n, sf['sB'], sf['sT'], sf['sG'],
                                sf['sM'], sf['sD'], sf['sA'], sub_off,
                                U, kp, dp, False, mp, C, vals, rvals,
                                kout, dout, yout, pout, True)
            data = np.bincount(idx_map.reshape(-1),
                               weights=vals.reshape(-1),
                               minlength=pattern.nnz + 1)[:-1]
            r0 = np.bincount(gloc.reshape(-1), weights=rvals.reshape(-1),
                             minlength=3 * n + 1)[:3 * n]
            # FD columns
            worst = 0.0
            for p in rng.choice(3 * n, 24, replace=False):
                h = 1e-7
                for sgn in (+1, -1):
                    Up = U.copy()
                    Up[p] += sgn * h
                    kernels.assemble_gd(ep, en, ef, n, sf['sB'], sf['sT'],
                                        sf['sG'], sf['sM'], sf['sD'],
                                        sf['sA'], sub_off,
                                        Up, kp, dp, False, mp, C,
                                        vals, rvals, kout, dout, yout, pout,
                                        False)
                    if sgn > 0:
                        rp = np.bincount(gloc.reshape(-1),
                                         weights=rvals.reshape(-1),
                                         minlength=3 * n + 1)[:3 * n]
                    else:
                        rm = np.bincount(gloc.reshape(-1),
                                         weights=rvals.reshape(-1),
                                         minlength=3 * n + 1)[:3 * n]
                fd = (rp - rm) / (2 * h)
                col = _column_from_data(data, pattern, gloc, vals, mesh, en, p)
                den = max(np.abs(fd).max(), np.abs(col).max(), 1e-12)
                worst = max(worst, np.abs(fd - col).max() / den)
            check(f"tangent gd/{name}/{eqn}", worst < 5e-4,
                  f"relerr={worst:.2e}")


def _column_from_data(data, pattern, gloc, vals, mesh, en, p):
    """extract J[:, p] by re-assembling the column from element data."""
    col = np.zeros(3 * mesh.nnode)
    # find all (e, a, b) with gloc[e,b] == p: sum vals[e,a,b] into gloc[e,a]
    ne = mesh.nelem
    # brute force over elements (test only)
    for e in range(ne):
        m = en[e]
        for b in range(3 * m):
            if gloc[e, b] == p:
                for a in range(3 * m):
                    col[gloc[e, a]] += vals[e, a, b]
    return col


def test_tangent_pf(m2, sf2):
    print("== FD tangent check: phase field ==")
    mesh, sf = m2, sf2
    n = mesh.nnode
    ep, en, ef = sf['elem_ptr'], sf['elem_n'], sf['elem_flat']
    pattern, idx_map, gloc = _build_pattern(mesh, sf)
    vals = np.zeros((mesh.nelem, 24, 24))
    rvals = np.zeros((mesh.nelem, 24))
    sub_off = np.zeros(mesh.nelem + 1, np.int64)
    np.cumsum(en, out=sub_off[1:])
    xy = mesh.xy
    mp = np.array([121.15, 80.77, 2.7e-3, 0.1])
    U = np.zeros(3 * n)
    U[0:2 * n:2] = 0.02 * xy[:, 0] + 0.004 * np.sin(3 * xy[:, 1])
    U[1:2 * n:2] = 0.01 * xy[:, 1] + 0.004 * np.cos(3 * xy[:, 0])
    U[2 * n:] = 0.3 + 0.4 * np.sin(2 * xy[:, 0]) ** 2
    ns = sf['nsub']
    Hp = 1e-4 + 1e-3 * rng.random(ns)
    Hout = np.zeros(ns)
    dout = np.zeros(ns)
    pout = np.zeros(ns)
    sout = np.zeros((ns, 3))
    kernels.assemble_pf(ep, en, ef, n, sf['sB'], sf['sT'], sf['sG'],
                        sf['sD'], sf['sA'], sub_off, U, Hp, False, mp,
                        vals, rvals, Hout, dout, pout, sout, True)
    data = np.bincount(idx_map.reshape(-1), weights=vals.reshape(-1),
                       minlength=pattern.nnz + 1)[:-1]
    worst = 0.0
    for p in rng.choice(3 * n, 24, replace=False):
        h = 1e-7
        rs = {}
        for sgn in (+1, -1):
            Up = U.copy()
            Up[p] += sgn * h
            kernels.assemble_pf(ep, en, ef, n, sf['sB'], sf['sT'], sf['sG'],
                                sf['sD'], sf['sA'], sub_off, Up, Hp, False, mp,
                                vals, rvals, Hout, dout, pout, sout, False)
            rs[sgn] = np.bincount(gloc.reshape(-1),
                                  weights=rvals.reshape(-1),
                                  minlength=3 * n + 1)[:3 * n]
        fd = (rs[+1] - rs[-1]) / (2 * h)
        col = _column_from_data(data, pattern, gloc, vals, mesh, en, p)
        den = max(np.abs(fd).max(), np.abs(col).max(), 1e-12)
        worst = max(worst, np.abs(fd - col).max() / den)
    check("tangent pf/AT2", worst < 5e-4, f"relerr={worst:.2e}")


# ----------------------------------------------------------------------
def test_spectral():
    print("== spectral split: stress/tangent vs FD ==")
    lam, mu = 121.15, 80.77
    worst_s = 0.0
    worst_t = 0.0
    for _ in range(400):
        exx = rng.normal(0, 0.03)
        eyy = rng.normal(0, 0.03)
        gxy = rng.normal(0, 0.05)
        sp, sm, Cp, Cm, psi = kernels.spectral(exx, eyy, gxy, lam, mu)
        h = 1e-7
        for k, (dex, dey, deg) in enumerate([(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
            args1 = (exx + h * dex, eyy + h * dey, gxy + h * deg, lam, mu)
            args2 = (exx - h * dex, eyy - h * dey, gxy - h * deg, lam, mu)
            sp1, sm1, _, _, _ = kernels.spectral(*args1)
            sp2, sm2, _, _, _ = kernels.spectral(*args2)
            fd_p = (sp1 - sp2) / (2 * h)
            fd_m = (sm1 - sm2) / (2 * h)
            worst_t = max(worst_t,
                          np.abs(fd_p - Cp[:, k]).max(),
                          np.abs(fd_m - Cm[:, k]).max())
        # energy consistency: psi = 0.5*(sp . eps+ ... ) check via dpsi/deps
        # psi derivatives should equal sigma+
        dpsi = np.array([0.0, 0.0, 0.0])
        for k, (dex, dey, deg) in enumerate([(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
            p1 = kernels.spectral(exx + h * dex, eyy + h * dey,
                                  gxy + h * deg, lam, mu)[4]
            p2 = kernels.spectral(exx - h * dex, eyy - h * dey,
                                  gxy - h * deg, lam, mu)[4]
            dpsi[k] = (p1 - p2) / (2 * h)
        worst_s = max(worst_s, np.abs(dpsi - sp).max())
    check("spectral tangent vs FD", worst_t < 1e-4, f"err={worst_t:.2e}")
    check("dpsi/deps = sigma+", worst_s < 1e-4, f"err={worst_s:.2e}")
    # known case: uniaxial strain tension -> full stress
    sp, sm, Cp, Cm, psi = kernels.spectral(0.01, 0.0, 0.0, lam, mu)
    check("spectral tension: sigma- = 0", np.abs(sm).max() < 1e-14)
    check("spectral tension: sigxx", abs(sp[0] - (lam + 2 * mu) * 0.01) < 1e-12)
    # both positive: Cp = full C, Cm = 0
    sp, sm, Cp, Cm, psi = kernels.spectral(0.02, 0.01, 0.005, lam, mu)
    Cfull = np.array([[lam + 2 * mu, lam, 0], [lam, lam + 2 * mu, 0],
                      [0, 0, mu]])
    check("spectral tt: Cp = C", np.abs(Cp - Cfull).max() < 1e-9)
    check("spectral tt: Cm = 0", np.abs(Cm).max() < 1e-9)
    # compression: Cp=0, Cm=C
    sp, sm, Cp, Cm, psi = kernels.spectral(-0.02, -0.01, 0.0, lam, mu)
    check("spectral cc: Cm = C", np.abs(Cm - Cfull).max() < 1e-9)
    check("spectral cc: Cp = 0", np.abs(Cp).max() < 1e-9)
    # mixed: exx>0 eyy<0
    sp, sm, Cp, Cm, psi = kernels.spectral(0.02, -0.01, 0.0, lam, mu)
    l1, l2 = 0.02, -0.01
    check("spectral mixed sp", abs(sp[0] - (lam * (l1 + l2 if l1 + l2 > 0 else 0)
                                            + 2 * mu * l1)) < 1e-10
          and abs(sm[1] - (lam * (l1 + l2 if l1 + l2 < 0 else 0)
                           + 2 * mu * l2)) < 1e-10)


# ----------------------------------------------------------------------
def test_damage_law():
    print("== damage law continuity ==")
    mp = np.zeros(21)
    mp[3] = 1e-4
    mp[4] = 0.7
    mp[5] = 1e4
    mp[6], mp[7] = 1.5, 5.0
    mp[8] = 0.999
    mp[13] = 0
    epsD = 1e-4
    for variant in (VAR_CGD, VAR_MNLD, VAR_SC):
        mp[11] = variant
        k_tr = epsD * 5.0
        k_fin = 1.5 * k_tr
        d1, _ = kernels.damage_law(k_tr * (1 - 1e-12), mp, variant)
        d2, _ = kernels.damage_law(k_tr * (1 + 1e-12), mp, variant)
        jump = abs(d2 - d1)
        d3, _ = kernels.damage_law(k_fin * (1 - 1e-12), mp, variant)
        d4, _ = kernels.damage_law(k_fin, mp, variant)
        cont = abs(d4 - d3)
        check(f"damage continuity at k_tr (v{variant})",
              jump < (0.01 if variant == VAR_MNLD else 1e-6),
              f"jump={jump:.2e}")
        check(f"damage continuity at k_fin (v{variant})", cont < 1e-9,
              f"jump={cont:.2e}")
        if variant == VAR_CGD:
            check(f"damage d(k_fin)<=dmax (v{variant})", d4 <= 0.999 + 1e-12)
        else:
            check(f"damage d(k_fin)=1 (v{variant})", abs(d4 - 1.0) < 1e-12)
        d0, _ = kernels.damage_law(epsD, mp, variant)
        check(f"damage d(epsD)=0 (v{variant})", d0 == 0.0)


# ----------------------------------------------------------------------
def test_problem_solve():
    print("== Problem: elastic solve on mini mesh ==")
    p = Problem(mini_case())
    rec = p.run(verbose=False)
    # elastic until onset; check final equilibrium residual
    rf, _, _ = p._free_system(p.U, want_tangent=False)
    check("mini run converges", len(rec['u']) > 1)
    check("mini residual small", np.linalg.norm(rf) < 1e-6,
          f"|r|={np.linalg.norm(rf):.2e}")
    # prescribed dofs respected
    U = p.U
    check("fix dofs zero", np.abs(U[p.fix_dofs]).max() < 1e-14)
    lam = p.lam
    check("load dofs = lam*v",
          np.abs(U[p.load_dofs] - lam * p.load_vals).max() < 1e-14)


if __name__ == '__main__':
    m2, sf2 = test_quadtree()
    test_patch(m2, sf2)
    test_spectral()
    test_damage_law()
    test_tangent_gd(m2, sf2)
    test_tangent_pf(m2, sf2)
    test_problem_solve()
    print()
    if FAIL:
        print("FAILURES:", FAIL)
        sys.exit(1)
    print("ALL TESTS PASSED")
