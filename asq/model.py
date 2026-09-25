"""
Problem class: BC handling, unified arc-length (UAL) solver with
process-zone weighted constraint (PZ-UAL), displacement-control NR,
predictor-corrector SDF-quadtree AMR, and state transfer.
"""
import time
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from . import kernels
from .kernels import VAR_CGD, VAR_MNLD, VAR_SC, MAXV, LMAX
from .sfem import build_sfem
from . import quadtree as qt

LMAX = 3 * MAXV


# ----------------------------------------------------------------------
def _build_pattern(mesh, sf):
    """CSR sparsity + per-element (a,b)->data-position map (vectorised)."""
    n = mesh.nnode
    ep, en, ef = sf['elem_ptr'], sf['elem_n'], sf['elem_flat']
    ne = mesh.nelem
    gloc = np.full((ne, LMAX), 3 * n, np.int64)      # pad -> dummy slot 3n
    rr, cc, ee, aa, bb = [], [], [], [], []
    for m in np.unique(en):
        sel = np.where(en == m)[0]
        k = len(sel)
        vids = np.empty((k, m), np.int64)
        for i, e in enumerate(sel):
            vids[i] = ef[ep[e]:ep[e] + m]
        gu = np.empty((k, 2 * m), np.int64)
        gu[:, 0::2] = 2 * vids
        gu[:, 1::2] = 2 * vids + 1
        G = np.hstack([gu, 2 * n + vids])            # (k, 3m)
        gloc[np.ix_(sel, np.arange(3 * m))] = G
        RR = np.repeat(G, 3 * m, axis=1)
        CC = np.tile(G, (1, 3 * m))
        rr.append(RR.ravel())
        cc.append(CC.ravel())
        loc = np.arange(3 * m)
        aa.append(np.tile(np.repeat(loc, 3 * m), k))
        bb.append(np.tile(np.tile(loc, 3 * m), k))
        ee.append(np.repeat(sel, 9 * m * m))
    rr = np.concatenate(rr)
    cc = np.concatenate(cc)
    ee = np.concatenate(ee)
    aa = np.concatenate(aa)
    bb = np.concatenate(bb)
    C = sp.coo_matrix((np.ones(len(rr)), (rr, cc)),
                      shape=(3 * n + 1, 3 * n + 1)).tocsr()
    C.sum_duplicates()
    # position of each generated (e,a,b) pair inside C.data
    ordk = np.lexsort((cc, rr))
    rr_s, cc_s = rr[ordk], cc[ordk]
    first = np.ones(len(rr_s), bool)
    first[1:] = (rr_s[1:] != rr_s[:-1]) | (cc_s[1:] != cc_s[:-1])
    pos = np.cumsum(first) - 1
    nnz = C.nnz
    idx_map = np.full((ne, LMAX, LMAX), nnz, np.int64)   # pad -> strip slot
    idx_map[ee[ordk], aa[ordk], bb[ordk]] = pos
    Cn = C[:3 * n, :3 * n]
    return Cn, idx_map, gloc


def _bary(p0x, p0y, p1x, p1y, p2x, p2y, x, y, tol=-1e-10):
    det = (p1x - p0x) * (p2y - p0y) - (p2x - p0x) * (p1y - p0y)
    if abs(det) < 1e-30:
        return None
    l1 = ((p1x - x) * (p2y - y) - (p2x - x) * (p1y - y)) / det
    l2 = ((p2x - x) * (p0y - y) - (p0x - x) * (p2y - y)) / det
    l3 = 1.0 - l1 - l2
    if l1 >= tol and l2 >= tol and l3 >= tol:
        return (l1, l2, l3)
    return None


class Problem:
    """Holds mesh + state + solvers for one model/case combination."""

    def __init__(self, case, opts=None):
        self.case = dict(case)
        if opts:
            self.case.update(opts)
        self.model = self.case['model']            # 'gd' or 'pf'
        self.variant = self.case.get('variant', VAR_SC)
        self.timing = dict(assemble=0.0, factor=0.0, adapt=0.0)
        self.nnewton = 0
        self.amr_count = 0
        self.snapshots = []
        self._build_initial()
        self._set_material()
        self._set_state_zero()

    # ------------------------------------------------------------------
    def _build_initial(self):
        c = self.case
        t = qt.Quadtree(c['x0'], c['y0'], c['x1'], c['y1'],
                        c['nx'], c['ny'], slits=c.get('slits', ()),
                        max_depth=c.get('max_depth', 14),
                        mask=c.get('mask'))
        t.align_slits()
        feats = c.get('init_features', [])
        if feats:
            items = []
            for f in feats:
                pts = np.asarray(f['pts'], float).reshape(-1, 2)
                items.append((pts, f.get('grade', 1.0),
                              f.get('r', c['h_min'])))
            hfun = qt.multi_feature_size_function(items, c['h_min'],
                                                  c['h_max'])
            t.refine_by_size_function(hfun)
        self.tree = t
        self._set_mesh()

    def _set_mesh(self):
        c = self.case
        self.mesh = qt.Mesh(self.tree)
        self.sf = build_sfem(self.mesh, beta=c.get('beta', 1.0))
        self.pattern, self.idx_map, self.gloc = _build_pattern(self.mesh,
                                                               self.sf)
        nnz = self.pattern.nnz
        self.vals = np.zeros((self.mesh.nelem, LMAX, LMAX))
        self.rvals = np.zeros((self.mesh.nelem, LMAX))
        self.idx_flat = self.idx_map.reshape(-1)
        self.ridx_flat = self.gloc.reshape(-1)
        # lumped scalar mass (PZ weights)
        n = self.mesh.nnode
        sM = self.sf['sM']
        colsum = sM.sum(axis=2).copy()            # (nsub, maxv)
        colsum[:, MAXV:] = 0.0
        ep, en, ef = self.sf['elem_ptr'], self.sf['elem_n'], self.sf['elem_flat']
        sub_off = np.zeros(self.mesh.nelem + 1, np.int64)
        np.cumsum(en, out=sub_off[1:])
        self.sub_off = sub_off
        Ml = np.zeros(n)
        for e in range(self.mesh.nelem):
            m = en[e]
            vids = ef[ep[e]:ep[e] + m]
            contrib = colsum[sub_off[e]:sub_off[e + 1], :m].sum(axis=0)
            Ml[vids] += contrib
        self.Mlump = Ml
        self._apply_bcs()

    def _set_material(self):
        c = self.case
        if self.model == 'gd':
            m = c['mat']
            lam = m['lam']
            if c.get('plane_stress'):
                # lambda' = 2*mu*lambda/(lambda+2mu)  (plane-stress reduction)
                lam = 2.0 * m['mu'] * lam / (lam + 2.0 * m['mu'])
            self.C = kernels.elastic_C(lam, m['mu'])
            self.mp = np.array([
                lam, m['mu'], 0.5 * m['lc'] ** 2, m['epsD'],
                m['alpha'], m['beta_law'], m.get('s1', 1.5),
                m.get('s2', 5.0), m.get('dmax', 0.999),
                m.get('n_fr', 100.0), m.get('ST', 1e-7),
                float(self.variant), float(m.get('eq_type', kernels.EQ_MAZARS)),
                float(m.get('law', kernels.LAW_MAZARS)),
                m.get('k0', 10.0), m.get('nu', 0.2),
                m.get('a_sc', 0.8), m.get('p_sc', 1.0), m.get('s_sc', 2.0),
                m.get('m_sc', 1000.0), m.get('kappa_s', 1e-6)])
        else:
            m = c['mat']
            lam = m['lam']
            if c.get('plane_stress'):
                lam = 2.0 * m['mu'] * lam / (lam + 2.0 * m['mu'])
            self.mp = np.array([lam, m['mu'], m['gc'], m['l']])
            # 2D AT2 critical energy (nucleation threshold)
            self.psi_c = 3.0 * m['gc'] / (16.0 * m['l'])

    def _set_state_zero(self):
        ns = self.sf['nsub']
        self.kappa = np.zeros(ns)
        self.d_sub = np.zeros(ns)
        self.d_prev = np.zeros(ns)
        self.H = np.zeros(ns)
        self.Y = np.zeros(ns)
        self.psi = np.zeros(ns)
        self.sig = np.zeros((ns, 3))
        # scratch history buffers: assemble() must NEVER mutate committed
        # history (line-search trials / failed attempts would otherwise
        # ratchet it irreversibly)
        self.kappa_s = np.zeros(ns)
        self.H_s = np.zeros(ns)
        self.U = self.set_U_prescribed(np.zeros(3 * self.mesh.nnode), 0.0)
        self.lam = 0.0

    # ------------------------------------------------------------------
    #  boundary conditions
    # ------------------------------------------------------------------
    def _sel_nodes(self, sel):
        xy = self.mesh.xy
        kind = sel[0]
        if kind == 'edge':
            x0, y0, x1, y1 = map(float, sel[1:5])
            L2 = (x1 - x0) ** 2 + (y1 - y0) ** 2
            t = ((xy[:, 0] - x0) * (x1 - x0) + (xy[:, 1] - y0) * (y1 - y0)) / L2
            px = x0 + t * (x1 - x0)
            py = y0 + t * (y1 - y0)
            d = np.hypot(xy[:, 0] - px, xy[:, 1] - py)
            tol = 1e-7 * max(np.sqrt(L2), 1.0)
            return np.where((d < tol) & (t > -1e-9) & (t < 1 + 1e-9))[0]
        if kind == 'point':
            d = np.hypot(xy[:, 0] - float(sel[1]), xy[:, 1] - float(sel[2]))
            i = int(np.argmin(d))
            return np.array([i])
        if kind == 'rect':
            x0, y0, x1, y1 = map(float, sel[1:5])
            return np.where((xy[:, 0] >= x0 - 1e-9) & (xy[:, 0] <= x1 + 1e-9)
                            & (xy[:, 1] >= y0 - 1e-9)
                            & (xy[:, 1] <= y1 + 1e-9))[0]
        raise ValueError(kind)

    def _apply_bcs(self):
        n = self.mesh.nnode
        comp = {'x': 0, 'y': 1}
        fix_dof, load_dof, load_val = [], [], []
        for sel, cp in self.case.get('fix', []):
            nodes = self._sel_nodes(sel)
            fix_dof.append(2 * nodes + comp[cp])
        for sel, cp, val in self.case.get('load', []):
            nodes = self._sel_nodes(sel)
            load_dof.append(2 * nodes + comp[cp])
            load_val.append(np.full(len(nodes), float(val)))
        fix_dof = np.concatenate(fix_dof) if fix_dof else np.zeros(0, int)
        load_dof = np.concatenate(load_dof) if load_dof else np.zeros(0, int)
        load_val = np.concatenate(load_val) if load_val else np.zeros(0)
        isfix = np.zeros(3 * n, bool)
        isfix[fix_dof] = True
        keep = ~isfix[load_dof]
        load_dof, load_val = load_dof[keep], load_val[keep]
        self.fix_dofs = np.unique(fix_dof)
        o = np.argsort(load_dof)
        self.load_dofs, self.load_vals = load_dof[o], load_val[o]
        self.presc = np.unique(np.concatenate([self.fix_dofs, self.load_dofs]))
        self.vpat = np.zeros(3 * n)
        self.vpat[self.load_dofs] = self.load_vals
        free = np.ones(3 * n, bool)
        free[self.presc] = False
        self.free = np.where(free)[0]
        self.nfree = len(self.free)
        self.perm = np.concatenate([self.free, self.presc])
        self.nf = self.nfree
        self.vpat_p = self.vpat[self.perm]
        self.v2 = float(self.vpat @ self.vpat)

    def set_U_prescribed(self, U, lam):
        U = U.copy()
        U[self.fix_dofs] = 0.0
        U[self.load_dofs] = lam * self.load_vals
        return U

    # ------------------------------------------------------------------
    #  assembly
    # ------------------------------------------------------------------
    def assemble(self, U, frozen=False, want_tangent=True, hist=None):
        t0 = time.perf_counter()
        sf = self.sf
        n = self.mesh.nnode
        if hist is None:
            kappa_p, d_prev, H_p = self.kappa, self.d_prev, self.H
        else:
            kappa_p, d_prev, H_p = hist
        if want_tangent:
            self.vals[:] = 0.0
        if self.model == 'gd':
            kernels.assemble_gd(sf['elem_ptr'], sf['elem_n'], sf['elem_flat'],
                                n, sf['sB'], sf['sT'], sf['sG'], sf['sM'],
                                sf['sD'], sf['sA'], self.sub_off,
                                U, kappa_p, d_prev, frozen, self.mp, self.C,
                                self.vals, self.rvals,
                                self.kappa_s, self.d_sub, self.Y, self.psi,
                                want_tangent)
        else:
            kernels.assemble_pf(sf['elem_ptr'], sf['elem_n'], sf['elem_flat'],
                                n, sf['sB'], sf['sT'], sf['sG'], sf['sD'], sf['sA'],
                                self.sub_off, U, H_p, frozen, self.mp,
                                self.vals, self.rvals,
                                self.H_s, self.d_sub, self.psi, self.sig,
                                want_tangent)
        r = np.bincount(self.ridx_flat, weights=self.rvals.reshape(-1),
                        minlength=3 * n + 1)[:3 * n]
        J = None
        if want_tangent:
            nnz = self.pattern.nnz
            data = np.bincount(self.idx_flat, weights=self.vals.reshape(-1),
                               minlength=nnz + 1)[:-1]
            J = sp.csr_matrix((data, self.pattern.indices,
                               self.pattern.indptr), shape=(3 * n, 3 * n))
        self.timing['assemble'] += time.perf_counter() - t0
        return r, J

    def commit_hist(self, U):
        """Commit history at an ACCEPTED state U (functional-history rule).

        Assemble evaluates max(committed, psi+(U)) into the scratch buffer;
        only here is the committed history advanced.  Derived fields
        (d_sub, psi, sig, Y) are refreshed at U in the same call.
        """
        self.assemble(U, want_tangent=False)
        if self.model == 'gd':
            self.kappa[:] = self.kappa_s
        else:
            self.H[:] = self.H_s
        self.d_prev[:] = self.d_sub

    def _free_system(self, U, frozen=False, want_tangent=True, hist=None):
        r, J = self.assemble(U, frozen, want_tangent, hist)
        if not want_tangent:
            return r[self.free], None, None
        Jp = J[self.perm][:, self.perm].tocsr()
        Jff = Jp[:self.nf, :self.nf].tocsc()
        q = Jp[:self.nf, self.nf:] @ self.vpat_p[self.nf:]
        return r[self.free], Jff, q

    def residual_norm(self, U, hist=None):
        rf, _, _ = self._free_system(U, want_tangent=False, hist=hist)
        return float(np.linalg.norm(rf))

    # ------------------------------------------------------------------
    #  displacement-control Newton (also used as fallback / rebalance)
    # ------------------------------------------------------------------
    def nr_fixed(self, lam_target, U0=None, lam0=None, tol=1e-8,
                 maxit=30, frozen=False, hist=None, commit=True):
        if U0 is None:
            U0, lam0 = self.U, self.lam
        dlam_total = lam_target - lam0
        nsub = 1
        for _ in range(30):
            ok = True
            U = U0.copy()
            for s in range(nsub):
                lam_s = lam0 + dlam_total * (s + 1) / nsub
                U = self.set_U_prescribed(U, lam_s)
                for it in range(maxit):
                    rf, Jff, q = self._free_system(U, frozen, True, hist)
                    rn = float(np.linalg.norm(rf))
                    if it == 0:
                        r0 = rn if rn > 1e-30 else 1e-30
                    if rn <= tol * r0:
                        break
                    try:
                        t0 = time.perf_counter()
                        lu = splu(Jff)
                        dy = lu.solve(-rf)
                        self.timing['factor'] += time.perf_counter() - t0
                    except Exception:
                        ok = False
                        break
                    best = None
                    for eta in (1.0, 0.5, 0.25, 0.1):
                        Ut = U.copy()
                        Ut[self.free] += eta * dy
                        if self.residual_norm(Ut, hist) < rn:
                            best = (eta, Ut)
                            break
                    if best is None:
                        ok = False
                        break
                    U = best[1]
                    self.nnewton += 1
                else:
                    ok = False
                if not ok:
                    break
            if ok:
                if commit and not frozen:
                    self.commit_hist(U)
                return U, lam_target, True
            nsub *= 2
            if nsub > 512:
                return U, lam0, False
        return U, lam0, False

    # ------------------------------------------------------------------
    #  unified arc-length increment
    # ------------------------------------------------------------------
    def ual_step(self, U0, lam0, Dy_prev, Dlam_prev, mode, w, tau, dl,
                 tol=1e-6, maxit=25):
        nf = self.nf
        N = nf + len(self.presc)
        # ---------------- predictor ----------------
        Dy = np.zeros(nf)
        Dlam = 0.0
        if Dy_prev is not None and np.linalg.norm(Dy_prev) > 0.0:
            if mode == 'pz':
                denom = float(w @ Dy_prev)
                s = tau / denom if denom > 1e-300 else 0.0
            else:
                nrm = np.sqrt(float(Dy_prev @ Dy_prev)
                              + Dlam_prev ** 2 * self.v2)
                s = dl / nrm if nrm > 0.0 else 0.0
            # guard: previous increment degenerate for this arc measure
            if not np.isfinite(s) or s <= 0.0 or s > 50.0:
                s = 0.0
            if s > 0.0:
                Dy = s * Dy_prev
                Dlam = s * Dlam_prev
        if np.linalg.norm(Dy) == 0.0 and Dlam == 0.0:
            seed = 1e-4 * max(self.case['lam_end'], 1e-12)
            if mode == 'pz' and tau > 0.0:
                seed = min(seed, tau)
            elif mode == 'sph' and dl > 0.0:
                seed = min(seed, dl)
            Dlam = seed
        # scale-aware constraint tolerance (avoids underflow death spiral)
        if mode == 'pz':
            gtol = 1e-6 * max(tau, 1e-12)
        else:
            gtol = 1e-6 * max(dl * dl, 1e-14)
        rpred = None
        U = U0
        lam = lam0
        it = 0
        for it in range(maxit):
            lam = lam0 + Dlam
            U = U0.copy()
            U[self.free] += Dy
            U = self.set_U_prescribed(U, lam)
            rf, Jff, q = self._free_system(U)
            rn = float(np.linalg.norm(rf))
            if rpred is None:
                rpred = max(rn, 1e-30)
            # constraint value
            if mode == 'pz':
                g = float(w @ Dy) - tau
            else:
                g = (float(Dy @ Dy) + Dlam * Dlam * self.v2) / N - dl * dl
            if (rn <= max(tol * rpred, 1e-11)) and abs(g) <= gtol:
                return U, lam, Dy, Dlam, it, True
            try:
                t0 = time.perf_counter()
                lu = splu(Jff)
                dyA = lu.solve(-rf)
                dyB = lu.solve(-q)
                self.timing['factor'] += time.perf_counter() - t0
            except Exception:
                return U, lam, Dy, Dlam, it, False
            if mode == 'pz':
                den = float(w @ dyB)
                dlam = -(g + float(w @ dyA)) / den if den != 0.0 else 0.0
            else:
                c0 = 2.0 / N
                num = g + c0 * float(Dy @ dyA)
                den = c0 * (float(Dy @ dyB) + Dlam * self.v2)
                dlam = -num / den if den != 0.0 else 0.0
            dy = dyA + dlam * dyB
            if not np.isfinite(dlam) or not np.all(np.isfinite(dy)):
                return U, lam, Dy, Dlam, it, False
            best = None
            for eta in (1.0, 0.5, 0.25, 0.1, 0.03, 0.01):
                Dy_t = Dy + eta * dy
                Dl_t = Dlam + eta * dlam
                lam_t = lam0 + Dl_t
                Ut = U0.copy()
                Ut[self.free] += Dy_t
                Ut = self.set_U_prescribed(Ut, lam_t)
                if self.residual_norm(Ut) < max(rn, 2e-11):
                    best = (Dy_t, Dl_t, Ut)
                    break
            if best is None:
                # residual already at machine precision and constraint
                # (linear in the increment) essentially satisfied
                if rn <= 1e-8 * rpred and abs(g) <= 1e-3 * max(tau, 1e-12):
                    return U, lam, Dy, Dlam, it, True
                return U, lam, Dy, Dlam, it, False
            Dy, Dlam, U = best
            self.nnewton += 1
        return U, lam0 + Dlam, Dy, Dlam, it, False

    # ------------------------------------------------------------------
    #  process zone weights / size field / AMR
    # ------------------------------------------------------------------
    def _sub_ebar(self, U=None):
        U = self.U if U is None else U
        n = self.mesh.nnode
        ebar = U[2 * n:]
        sD = self.sf['sD']
        out = np.zeros(self.sf['nsub'])
        ep, en, ef = (self.sf['elem_ptr'], self.sf['elem_n'],
                      self.sf['elem_flat'])
        for e in range(self.mesh.nelem):
            m = en[e]
            vids = ef[ep[e]:ep[e] + m]
            vals = ebar[vids]
            s0, s1 = self.sub_off[e], self.sub_off[e + 1]
            out[s0:s1] = sD[s0:s1, :m] @ vals
        return out

    def _pz_mask(self):
        """Active process zone: damaging annulus + elastic PZ cloud.

        Fully broken subcells (d >= d_dead) are EXCLUDED: they no longer
        respond to the load, so including them makes the PZ arc-length
        constraint degenerate (w @ dyB -> 0) once a crack forms -- the
        arc must measure the ACTIVE tip, not the dead band.
        """
        c = self.case
        if self.model == 'gd':
            theta = c.get('theta', 0.9)
            d_thr = c.get('d_thr', 1e-3)
            d_dead = c.get('d_dead', 0.95 * self.mp[8])
            eb = self._sub_ebar()
            return (((eb >= theta * self.mp[3]) | (self.d_sub >= d_thr))
                    & (self.d_sub < d_dead))
        d_thr = c.get('d_thr', 0.15)
        d_dead = c.get('d_dead', 0.95)
        psi_frac = c.get('psi_frac', 0.5)
        return (((self.d_sub >= d_thr) & (self.d_sub < d_dead))
                | ((self.psi >= psi_frac * self.psi_c)
                   & (self.d_sub < d_dead)))

    def size_field(self):
        c = self.case
        pts = self.sf['sC'][self._pz_mask()]
        if len(pts) == 0:
            feats = c.get('init_features', [])
            if feats:
                pts = np.vstack([np.asarray(f['pts'], float).reshape(-1, 2)
                                 for f in feats])
            else:
                return None
        return qt.feature_size_function(pts, c['h_min'], c['h_max'],
                                        c.get('grade', 1.0),
                                        c.get('r_buf', c['h_min']))

    def adapt(self, U0, lam0, U_trial=None):
        """Predictor-corrector AMR. Returns (changed, Dy_transferred)."""
        t0 = time.perf_counter()
        hfun = self.size_field()
        changed = False
        Dy_trans = None
        if hfun is not None and self.tree.refine_by_size_function(hfun) > 0:
            changed = True
            old = dict(U=U0.copy(), kappa=self.kappa.copy(),
                       d_prev=self.d_prev.copy(), H=self.H.copy(),
                       sf=self.sf, mesh=self.mesh, sub_off=self.sub_off)
            mesh_old = self.mesh
            self._set_mesh()
            self._transfer(old, mesh_old)
            self.amr_count += 1
            # frozen re-balance at the step start state
            hist = (self.kappa, self.d_prev, self.H)
            Ub, _, ok = self.nr_fixed(lam0, U0=self.U, lam0=lam0,
                                      frozen=True, hist=hist, maxit=12,
                                      tol=1e-7)
            if ok:
                self.U = Ub
            if U_trial is not None:
                Ut = self._interp_nodal(old, mesh_old, U_trial)
                Dy_trans = (Ut - self.U)[self.free].copy()
        self.timing['adapt'] += time.perf_counter() - t0
        return changed, Dy_trans

    def _transfer(self, old, mesh_old):
        """Interpolate nodal fields + inherit subcell histories."""
        sf_old = old['sf']
        mo = mesh_old
        no = mo.nnode
        Uo = old['U']
        ef, ep, en = sf_old['elem_flat'], sf_old['elem_ptr'], sf_old['elem_n']
        eidx = mo.elem_index_of_key()
        w_old = sf_old['eW']
        xy_old = mo.xy
        # --- new vertex -> incident element (hint for interpolation) ---
        inc = {}
        for e, poly in enumerate(self.mesh.elems):
            for v in poly:
                if v not in inc:
                    inc[v] = e
        # --- old coordinate -> old vid fast path (non-slit vertices) ---
        q = mo._quant
        oldmap = {}
        for vi in range(no):
            x, y = xy_old[vi]
            if mo._vertex_on_slit(x, y)[0] >= 0:
                continue
            oldmap[(int(round(x / q)), int(round(y / q)))] = vi

        def old_ancestor(e_new):
            key = self.mesh.elem_keys[e_new]
            while key not in eidx:
                lvl, ix, iy = key
                key = (lvl - 1, ix // 2, iy // 2)
                if lvl == 0:
                    return None
            return eidx[key]

        def interp_in(e_old, x, y, val_full):
            m = en[e_old]
            vids = ef[ep[e_old]:ep[e_old] + m]
            xyv = xy_old[vids]
            cx = xyv[:, 0].mean()
            cy = xyv[:, 1].mean()
            w = w_old[e_old, :m]
            vals = val_full[vids]
            vc = float(w @ vals)
            for k in range(m):
                k1 = (k + 1) % m
                b = _bary(xyv[k, 0], xyv[k, 1], xyv[k1, 0], xyv[k1, 1],
                          cx, cy, x, y)
                if b is not None:
                    return b[0] * vals[k] + b[1] * vals[k1] + b[2] * vc
            # numerical edge: nearest vertex
            d = np.hypot(xyv[:, 0] - x, xyv[:, 1] - y)
            return float(vals[int(np.argmin(d))])

        nnew = self.mesh.nnode
        xy = self.mesh.xy
        U = np.zeros(3 * nnew)
        qn = self.mesh._quant
        for v in range(nnew):
            x, y = xy[v]
            if self.mesh._vertex_on_slit(x, y)[0] < 0:
                key = (int(round(x / qn)), int(round(y / qn)))
                vi = oldmap.get(key)
                if vi is not None:
                    U[2 * v] = Uo[2 * vi]
                    U[2 * v + 1] = Uo[2 * vi + 1]
                    U[2 * nnew + v] = Uo[2 * no + vi]
                    continue
            e_old = old_ancestor(inc[v])
            if e_old is None:
                continue
            U[2 * v] = interp_in(e_old, x, y, Uo[0:2 * no:2])
            U[2 * v + 1] = interp_in(e_old, x, y, Uo[1:2 * no:2])
            U[2 * nnew + v] = interp_in(e_old, x, y, Uo[2 * no:])
        self.U = U
        self._interp_nodal_post(old, mo)

    def _interp_nodal_post(self, old, mo):
        """allocate + inherit subcell histories on the new mesh."""
        ns = self.sf['nsub']
        kap = np.zeros(ns)
        dprev = np.zeros(ns)
        H = np.zeros(ns)
        ef, ep, en = (self.sf['elem_flat'], self.sf['elem_ptr'],
                      self.sf['elem_n'])
        ef_o, ep_o, en_o = (old['sf']['elem_flat'], old['sf']['elem_ptr'],
                            old['sf']['elem_n'])
        xy_old = mo.xy
        eidx = mo.elem_index_of_key()
        w_old = old['sf']['eW']
        kold, dold, Hold = old['kappa'], old['d_prev'], old['H']
        sub_off_o = old['sub_off']
        sC = self.sf['sC']
        for e in range(self.mesh.nelem):
            key = self.mesh.elem_keys[e]
            while key not in eidx:
                lvl, ix, iy = key
                key = (lvl - 1, ix // 2, iy // 2)
                if lvl == 0:
                    key = None
                    break
            if key is None:
                continue
            e_old = eidx[key]
            m = en_o[e_old]
            vids = ef_o[ep_o[e_old]:ep_o[e_old] + m]
            xyv = xy_old[vids]
            cx = xyv[:, 0].mean()
            cy = xyv[:, 1].mean()
            s0, s1 = self.sub_off[e], self.sub_off[e + 1]
            for si in range(s0, s1):
                x, y = sC[si]
                for k in range(m):
                    k1 = (k + 1) % m
                    b = _bary(xyv[k, 0], xyv[k, 1], xyv[k1, 0], xyv[k1, 1],
                              cx, cy, x, y)
                    if b is not None:
                        so = sub_off_o[e_old] + k
                        kap[si] = kold[so]
                        dprev[si] = dold[so]
                        H[si] = Hold[so]
                        break
        self.kappa = kap
        self.d_prev = dprev
        self.H = H
        self.d_sub = np.zeros(ns)
        self.Y = np.zeros(ns)
        self.psi = np.zeros(ns)
        self.sig = np.zeros((ns, 3))
        self.kappa_s = np.zeros(ns)
        self.H_s = np.zeros(ns)

    def _interp_nodal(self, old, mo, U_trial):
        """interpolate a full nodal vector onto the current mesh."""
        no = mo.nnode
        Uo = U_trial
        ef, ep, en = (old['sf']['elem_flat'], old['sf']['elem_ptr'],
                      old['sf']['elem_n'])
        eidx = mo.elem_index_of_key()
        w_old = old['sf']['eW']
        xy_old = mo.xy
        inc = {}
        for e, poly in enumerate(self.mesh.elems):
            for v in poly:
                if v not in inc:
                    inc[v] = e
        q = mo._quant
        oldmap = {}
        for vi in range(no):
            x, y = xy_old[vi]
            if mo._vertex_on_slit(x, y)[0] >= 0:
                continue
            oldmap[(int(round(x / q)), int(round(y / q)))] = vi

        def interp_in(e_old, x, y, val_full):
            m = en[e_old]
            vids = ef[ep[e_old]:ep[e_old] + m]
            xyv = xy_old[vids]
            cx = xyv[:, 0].mean()
            cy = xyv[:, 1].mean()
            w = w_old[e_old, :m]
            vals = val_full[vids]
            vc = float(w @ vals)
            for k in range(m):
                k1 = (k + 1) % m
                b = _bary(xyv[k, 0], xyv[k, 1], xyv[k1, 0], xyv[k1, 1],
                          cx, cy, x, y)
                if b is not None:
                    return b[0] * vals[k] + b[1] * vals[k1] + b[2] * vc
            d = np.hypot(xyv[:, 0] - x, xyv[:, 1] - y)
            return float(vals[int(np.argmin(d))])

        def old_ancestor(e_new):
            key = self.mesh.elem_keys[e_new]
            while key not in eidx:
                lvl, ix, iy = key
                key = (lvl - 1, ix // 2, iy // 2)
                if lvl == 0:
                    return None
            return eidx[key]

        nnew = self.mesh.nnode
        xy = self.mesh.xy
        U = np.zeros(3 * nnew)
        qn = self.mesh._quant
        for v in range(nnew):
            x, y = xy[v]
            if self.mesh._vertex_on_slit(x, y)[0] < 0:
                key = (int(round(x / qn)), int(round(y / qn)))
                vi = oldmap.get(key)
                if vi is not None:
                    U[2 * v] = Uo[2 * vi]
                    U[2 * v + 1] = Uo[2 * vi + 1]
                    U[2 * nnew + v] = Uo[2 * no + vi]
                    continue
            e_old = old_ancestor(inc[v])
            if e_old is None:
                continue
            U[2 * v] = interp_in(e_old, x, y, Uo[0:2 * no:2])
            U[2 * v + 1] = interp_in(e_old, x, y, Uo[1:2 * no:2])
            U[2 * nnew + v] = interp_in(e_old, x, y, Uo[2 * no:])
        return U

    # ------------------------------------------------------------------
    #  driver
    # ------------------------------------------------------------------
    def _onset_ratio(self):
        if self.model == 'gd':
            return float(self._sub_ebar().max() / self.mp[3])
        return float(self.H.max() / self.psi_c)

    def damage_active(self):
        thr = self.case.get('onset_d', 1e-3)
        return float(self.d_sub.max()) > thr

    def _pz_weights(self):
        w = np.zeros(self.nfree)
        fmap = -np.ones(3 * self.mesh.nnode, np.int64)
        fmap[self.free] = np.arange(self.nfree)
        mask = self._pz_mask()
        ef, ep, en = (self.sf['elem_flat'], self.sf['elem_ptr'],
                      self.sf['elem_n'])
        n = self.mesh.nnode
        for si in np.where(mask)[0]:
            e = self.sf['sub_el'][si]
            for v in ef[ep[e]:ep[e] + en[e]]:
                j = fmap[2 * n + int(v)]
                if j >= 0:
                    w[j] = self.Mlump[int(v)]
        if w.sum() <= 0.0:
            w[:] = 1.0
        return w

    def _record(self, rec, lam, t_start, iters):
        n = self.mesh.nnode
        r, _ = self.assemble(self.U, want_tangent=False)
        F = float(np.abs(r[self.load_dofs]).sum()) if len(self.load_dofs) \
            else 0.0
        Ff = float(np.abs(r[self.fix_dofs]).sum()) if len(self.fix_dofs) \
            else 0.0
        Rxf = float(np.abs(r[self.fix_dofs[self.fix_dofs % 2 == 0]]).sum())
        Ryf = float(np.abs(r[self.fix_dofs[self.fix_dofs % 2 == 1]]).sum())
        rec['lam'].append(lam)
        rec['u'].append(lam * self.case.get('u_scale', 1.0))
        rec['F'].append(F)
        rec['Ffix'].append(Ff)
        rec['Rx'].append(Rxf)
        rec['Ry'].append(Ryf)
        rec['dof'].append(3 * n)
        rec['nelem'].append(self.mesh.nelem)
        rec['iters'].append(iters)
        rec['amr'].append(self.amr_count)
        rec['t'].append(time.perf_counter() - t_start)
        rec['maxd'].append(float(self.d_sub.max()) if len(self.d_sub) else 0.0)

    def _save_partial(self, rec):
        """crash/kill-safe record dump (best effort)."""
        try:
            import json as _json
            name = self.case.get('name', 'problem')
            with open(f'results/{name}_rec_partial.json', 'w') as f:
                _json.dump({k: list(map(float, v))
                            for k, v in rec.items()
                            if isinstance(v, list)}, f)
        except Exception:
            pass

    def run(self, verbose=True, record_every=1):
        c = self.case
        rec = dict(lam=[], u=[], F=[], Ffix=[], Rx=[], Ry=[], dof=[],
                   nelem=[], iters=[], amr=[], t=[], maxd=[])
        t_start = time.perf_counter()
        wall_max = c.get('wall_max_s', np.inf)
        lam_end = c['lam_end']
        lam = 0.0
        step = 0
        max_steps = c.get('max_steps', 600)
        # ------------- phase A: displacement ramp to damage onset ----------
        dlam = c.get('dlam0', 0.02 * lam_end)
        onset = False
        fails = 0
        q_prev = 0.0
        handover = None
        while lam < lam_end and step < max_steps:
            step += 1
            target = min(lam + dlam, lam_end)
            U_prev, lam_prev = self.U, lam
            Uc, lamc, ok = self.nr_fixed(target, U0=self.U, lam0=lam,
                                         tol=1e-9)
            if not ok:
                dlam *= 0.25
                fails += 1
                if fails > 30:
                    break
                step -= 1
                continue
            self.U, lam = Uc, lamc
            self.lam = lam
            self._record(rec, lam, t_start, 0)
            # AMR pre-nucleation (process-zone cloud grows before damage)
            if c.get('amr', True):
                ch, _ = self.adapt(self.U, lam)
            else:
                ch = False
            if ch:
                self.lam = lam
                handover = None       # increment stale after remeshing
            else:
                handover = (U_prev, lam_prev)
            q = self._onset_ratio()
            dq = q - q_prev
            q_prev = q
            dmax = float(self.d_sub.max()) if len(self.d_sub) else 0.0
            if dmax > c.get('onset_d', 5e-3) or q > 1.2 or lam >= lam_end:
                onset = True
                break
            if q < 0.5:
                dlam = min(dlam * 2.0, 0.05 * lam_end)
            elif dq > 0.2:
                dlam *= 0.2 / max(dq, 0.02)
        # ------------- phase B: process-zone weighted UAL -------------
        if onset and lam < lam_end:
            w = self._pz_weights()
            Dy_prev = None
            Dlam_prev = None
            # seed the arc length and predictor from the last phase-A step
            if handover is not None:
                U_prev, lam_prev = handover
                dU = (self.U - U_prev)[self.free]
                dlam_last = lam - lam_prev
                arc = float(w @ dU)
                if arc > 0.0 and dlam_last > 0.0:
                    Dy_prev = dU
                    Dlam_prev = dlam_last
            # seed tau from the last phase-A PZ increment when available
            if Dy_prev is not None:
                tau = max(float(w @ Dy_prev), 1e-14)
            else:
                tau = c.get('tau0', 1e-4)
            tau = max(tau, 1e-14)
            tau_max = c.get('tau_max', np.inf)
            it_min = c.get('it_min', 4)
            it_max = c.get('it_max', 12)
            fails = 0
            Fpeak = max(rec['F']) if rec['F'] else 0.0
            stall = 0
            lam_stall = -1.0
            F_stall = -1.0
            while step < max_steps and lam < lam_end:
                if time.perf_counter() - t_start > wall_max:
                    if verbose:
                        print('    [wall-clock budget reached: terminating]')
                    break
                step += 1
                ok = False
                Dy = Dlam = None
                iters = 0
                dl_s = None
                for attempt in range(14):
                    U0 = self.U.copy()
                    lam0 = lam
                    w = self._pz_weights()
                    if attempt < 8:
                        U, lam_n, Dy, Dlam, iters, ok = self.ual_step(
                            U0, lam0, Dy_prev, Dlam_prev, 'pz', w, tau, 0.0)
                    else:
                        # spherical arc-length fallback: constraint needs no
                        # process-zone weights, rounds limit/snapback points
                        # where the PZ measure degenerates (UAL paper design)
                        if dl_s is None:
                            if Dy_prev is not None:
                                dl_s = 0.5 * np.sqrt(
                                    float(Dy_prev @ Dy_prev)
                                    + self.v2 * Dlam_prev ** 2)
                            else:
                                dl_s = 1e-3 * np.sqrt(self.v2) \
                                    * max(lam_end, 1e-12)
                            dl_s = max(dl_s, 1e-14)
                        U, lam_n, Dy, Dlam, iters, ok = self.ual_step(
                            U0, lam0, Dy_prev, Dlam_prev, 'sph', w, 0.0,
                            dl_s)
                    if not ok:
                        if attempt < 8:
                            tau = max(tau * 0.5, 1e-14 * max(tau, 1e-30))
                        else:
                            dl_s = max(dl_s * 0.5, 1e-14 * max(dl_s, 1e-30))
                        fails += 1
                        continue
                    # root filter: reject unphysical damage leaps (the
                    # total-collapse branch signature).  Relative-increment
                    # filters are WRONG near limit points, where the load
                    # direction response |dyB| legitimately becomes huge.
                    nnd = self.mesh.nnode
                    d_jump = float((U[2 * nnd:] - U0[2 * nnd:]).max())
                    if d_jump > 0.5:
                        tau = max(tau * 0.5, 1e-14 * max(tau, 1e-30))
                        fails += 1
                        continue
                    # provisional accept + AMR predictor-corrector
                    self.U = U
                    lam = lam_n
                    self.lam = lam
                    if c.get('amr', True):
                        ch, Dy_t = self.adapt(U0, lam0, U)
                        if ch:
                            lam = lam0
                            self.lam = lam0
                            if Dy_t is not None:
                                Dy_prev = Dy_t
                                Dlam_prev = Dlam
                            else:
                                Dy_prev = None
                                Dlam_prev = None
                            # the step is NOT accepted: the mesh changed, so
                            # the old-mesh increment must not survive attempt-
                            # budget exhaustion into the accept path
                            ok = False
                            Dy = Dlam = None
                            continue
                    ok = True
                    break
                if not ok:
                    # displacement-control fallback; re-seed the arc length
                    U0 = self.U.copy()
                    lam0 = lam
                    # w may be stale if the last attempt ended in an AMR
                    # remesh (attempt budget exhausted on the retry)
                    w = self._pz_weights()
                    if Dlam_prev is not None and Dlam_prev > 0.0:
                        step_lam = 0.5 * abs(Dlam_prev)
                    else:
                        step_lam = 0.01 * lam_end
                    step_lam = min(step_lam, 0.05 * lam_end)
                    Uc, lamc, ok2 = self.nr_fixed(min(lam + step_lam, lam_end),
                                                  tol=1e-7)
                    if ok2:
                        self.U, lam = Uc, lamc
                        self.lam = lam
                        dU = (self.U - U0)[self.free]
                        arc = float(w @ dU)
                        if arc > 0.0:
                            Dy_prev = dU
                            Dlam_prev = lam - lam0
                            tau = min(0.5 * arc, 0.2 * w.sum())
                        else:
                            Dy_prev = None
                            Dlam_prev = None
                        fails = 0
                        self._record(rec, lam, t_start, 0)
                        Ffb = rec['F'][-1]
                        Fpeak = max(Fpeak, Ffb)
                        if lam >= lam_end:
                            break
                        if len(rec['F']) > 12 and Ffb < c.get(
                                'f_end_frac', 0.02) * Fpeak:
                            break
                        if abs(lam - lam_stall) < 1e-10 * lam_end \
                                and abs(Ffb - F_stall) < 1e-6 * max(Fpeak, 1e-30):
                            stall += 1
                            if stall > 20:
                                if verbose:
                                    print('    [stall detected in fallback: '
                                          'terminating]')
                                break
                        else:
                            stall = 0
                            lam_stall = lam
                            F_stall = Ffb
                        continue
                    break
                # accept increment
                self.commit_hist(self.U)
                Dy_prev, Dlam_prev = Dy, Dlam
                if attempt >= 8:
                    # came via spherical mode: re-seed the PZ arc length
                    arc = float(w @ Dy)
                    if arc > 0.0:
                        tau = min(max(arc, 1e-14), 0.2 * max(w.sum(), 1e-30))
                # stall guard: state frozen (degenerate PZ / singular band)
                if abs(lam - lam_stall) < 1e-10 * lam_end \
                        and abs(rec['F'][-1] - F_stall) < 1e-6 * max(Fpeak, 1e-30):
                    stall += 1
                else:
                    stall = 0
                    lam_stall = lam
                    F_stall = rec['F'][-1]
                if stall > 20:
                    if verbose:
                        print('    [stall detected: state frozen, terminating]')
                    break
                fails = 0
                self._record(rec, lam, t_start, iters)
                if step % 10 == 0:
                    self._save_partial(rec)
                if verbose and step % 10 == 0:
                    print(f"    step {step}: lam={lam:.4g} "
                          f"F={rec['F'][-1]:.4g} dof={rec['dof'][-1]} "
                          f"it={iters} tau={tau:.3g} amr={self.amr_count} "
                          f"t={rec['t'][-1]:.1f}s")
                F = rec['F'][-1]
                Fpeak = max(Fpeak, F)
                # step size control by Newton count
                if iters <= it_min:
                    tau = min(tau * 10 ** 0.2, tau_max, 0.2 * w.sum())
                elif iters >= it_max:
                    tau *= 10 ** -0.2
                # termination
                if lam >= lam_end:
                    break
                if len(rec['F']) > 12 and F < c.get('f_end_frac', 0.02) * Fpeak:
                    break
        self._record(rec, lam, t_start, 0)
        rec['walltime'] = time.perf_counter() - t_start
        rec['timing'] = dict(self.timing)
        rec['amr_count'] = self.amr_count
        rec['nnewton'] = self.nnewton
        self.rec = rec
        if verbose:
            print(f"  steps={len(rec['u'])} dof_end={rec['dof'][-1]} "
                  f"amr={self.amr_count} newton={self.nnewton} "
                  f"t={rec['walltime']:.1f}s "
                  f"t_asm={rec['timing']['assemble']:.1f}s "
                  f"t_fac={rec['timing']['factor']:.1f}s")
        return rec
