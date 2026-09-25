"""1D calibration of the gradient-damage variants on the Jin material.

Targets (plane-strain homogeneous, consistent with the 2D code):
  sigma_c = 0.3247*sqrt(Ebar*gc/l_pf)  (Ebar = lambda+2mu = 282.69 kN/mm^2)
          = 3.2756 kN/mm^2 (3276 MPa)   [AT2 homogeneous strength]
  Gf      = gc = 2.7 N/mm               [AT2 fracture energy]

Calibrated parameters:
  epsD  <- homogeneous peak stress (Mazars-type: peak at onset, epsD ~ eps_c)
  lc    <- localized fracture energy of a long 1D bar at the paper's
           transition shape (s1=1.5, s2=9): with sigma_c = 3.28 kN/mm^2 and
           Gf = gc = 2.7 N/mm the critical opening is tiny, so the
           nonlocal length must exceed the PF length to dissipate gc
           (result: lc ~ 5.5*l_pf -> GD band ~5x wider than AT2 band,
           i.e. GD is resolvable on a ~10x coarser mesh).
  The 1D bar uses the implicit-gradient averaging, SAME convention as
  asq/kernels.py: int(eb*deb) + c*int(grad eb.grad deb) = int(fr*eq*deb),
  c = lc^2/2.

Shape parameters (dimensionless, from the MNLD paper's reference setup):
  MNLD/SC: alpha=0.96, beta*epsD=0.15, s1=1.5, s2=9, dmax=0.999
           (+ SC state function a=0.8, p=1, s=2, m=1000)
  CGD    : alpha=0.7, beta*epsD=2 (UAL-paper style residual strength);
           Gf is UNBOUNDED for CGD (sigma -> (1-alpha)*sigma_c plateau),
           so lc is set equal to the MNLD value for spatial-scale parity.

Bar solution (secant + band-opening arc length):
  eps(x) = sigma*s(x),  s = 1/(Ebar*g_eff(d(kappa(x))))
  ebar   = sigma*(I - c*D2)^-1 s     (FD, Neumann BCs, tridiagonal)
  constraint: opening of the central window = w_target
  history: kappa <- max(kappa_committed, ebar) (damped fixed point)
  Gf (energy form) = int_x D(kappa_f(x)) dx,
    D(kappa) = int Y_out(k') * dd(k') dk'   (exact dissipated energy,
    converges once damage saturates; unaffected by CGD residual plateau)

Usage: python calib1d.py [cgd|mnld|sc ...]   (default: all three)
Writes results/calib1d.json + results/calib1d_<variant>.png
"""
import sys
import os
import json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.linalg import solve_banded
from numba import njit

from asq import benchmarks as B
from asq import kernels as K

# Jin 2024 benchmarks are PLANE STRESS (repo: scenario 3/4, "Plane stress
# = yes"); the 1D homogeneous bar uses the uniaxial plane-stress relation
# sigma = E*eps, psi+ = E*eps^2/2 (verified: sigma_2 = 0 exactly).
PLANE_STRESS = True
E_PS = B.JIN_MU * (3 * B.JIN_LAM + 2 * B.JIN_MU) / (B.JIN_LAM + B.JIN_MU)
EBAR = E_PS if PLANE_STRESS else B.JIN_EP
GC = B.JIN_GC                      # 2.7e-3 kN/mm
SIGC = (0.3247 * np.sqrt(EBAR * B.JIN_GC / B.JIN_L)
        if PLANE_STRESS else B.JIN_SIGC)
VARIANTS = {'cgd': K.VAR_CGD, 'mnld': K.VAR_MNLD, 'sc': K.VAR_SC}

# dimensionless shape settings per variant
SHAPES = {
    # beta*epsD is set steep (peak at damage onset) so that the pre-peak
    # response stays elastic like AT2; a gentle beta (paper's 0.15) puts the
    # homogeneous peak at ~6.7*epsD and the whole specimen dissipates before
    # localization (Gf then unrelated to lc).
    'cgd': dict(alpha=0.7, beta_epsD=2.0, s1=1.5, s2=9.0, dmax=0.999,
                a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0),
    'mnld': dict(alpha=0.96, beta_epsD=4.0, s1=1.5, s2=9.0, dmax=0.999,
                 a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0),
    'sc': dict(alpha=0.96, beta_epsD=4.0, s1=1.5, s2=9.0, dmax=0.999,
               a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0),
}


def mp_array(name, epsD, lc=None, s2=None):
    m = SHAPES[name]
    if lc is None:
        lc = B.JIN_L
    mp = np.zeros(24)
    mp[3] = epsD
    mp[4] = m['alpha']
    mp[5] = m['beta_epsD'] / epsD           # beta * epsD = const (shape)
    mp[6] = m['s1']
    mp[7] = m['s2'] if s2 is None else s2
    mp[8] = m['dmax']
    mp[9] = 100.0
    mp[10] = 1e-7
    mp[12] = K.EQ_MAZARS
    mp[13] = K.LAW_MAZARS
    mp[14] = 10.0
    mp[15] = 0.2
    mp[16] = m['a_sc']
    mp[17] = m['p_sc']
    mp[18] = m['s_sc']
    mp[19] = m['m_sc']
    mp[20] = 1e-8 if name == 'mnld' else 1e-6
    mp[2] = 0.5 * lc * lc
    return mp


# ---------------- numba hot loops ----------------
@njit(cache=True)
def _secant_profile(kappa, d_prev, epsD_x, mp, variant, Emod):
    n = kappa.shape[0]
    s = np.empty(n)
    for i in range(n):
        m2 = mp.copy()
        m2[3] = epsD_x[i]
        d, dd = K.damage_law(kappa[i], m2, variant)
        g, dg = K.degradation(d, d_prev[i], m2, variant)
        s[i] = 1.0 / max(Emod * g, 1e-14)
    return s


@njit(cache=True)
def _damage_vec(kappa, epsD_x, mp, variant):
    n = kappa.shape[0]
    d = np.empty(n)
    for i in range(n):
        m2 = mp.copy()
        m2[3] = epsD_x[i]
        d[i] = K.damage_law(kappa[i], m2, variant)[0]
    return d


@njit(cache=True)
def _sigma_hom(kappa, mp, variant, Emod):
    n = kappa.shape[0]
    sig = np.empty(n)
    for i in range(n):
        d, dd = K.damage_law(kappa[i], mp, variant)
        g, dg = K.degradation(d, d, mp, variant)
        sig[i] = g * Emod * kappa[i]
    return sig


@njit(cache=True)
def _diss_density(kappa, mp, variant, Emod):
    """W(kappa) = Y_out * dd  (dissipation rate per unit kappa per volume)."""
    n = kappa.shape[0]
    W = np.empty(n)
    for i in range(n):
        d, dd = K.damage_law(kappa[i], mp, variant)
        g, dg = K.degradation(d, d, mp, variant)
        psi0 = 0.5 * Emod * kappa[i] * kappa[i]
        if variant == K.VAR_MNLD:
            Y = (1.0 - d) * psi0
        else:
            Y = -dg * psi0
        W[i] = Y * dd
    return W


# ---------------- homogeneous response ----------------
def peak_hom(mp, variant, k_hi=None):
    epsD = mp[3]
    if k_hi is None:
        k_hi = epsD * (mp[6] * mp[7] * 1.05 if variant != K.VAR_CGD else 30.0)
    ks = np.linspace(epsD, k_hi, 12000)
    s = _sigma_hom(ks, mp, variant, EBAR)
    i = int(np.argmax(s))
    lo = ks[max(i - 1, 0)]
    hi = ks[min(i + 1, len(ks) - 1)]
    ks2 = np.linspace(lo, hi, 400)
    s2 = _sigma_hom(ks2, mp, variant, EBAR)
    j = int(np.argmax(s2))
    return float(s2[j]), float(ks2[j])


def solve_epsD(name, variant):
    a, b = 0.2 * SIGC / EBAR, 5.0 * SIGC / EBAR

    def f(e):
        return peak_hom(mp_array(name, e), variant)[0] - SIGC
    fa = f(a)
    for _ in range(60):
        m = 0.5 * (a + b)
        fm = f(m)
        if abs(fm) < 1e-10:
            return m
        if fa * fm <= 0:
            b = m
        else:
            a, fa = m, fm
    return 0.5 * (a + b)


# ---------------- 1D bar ----------------
class Bar1D:
    def __init__(self, mp, variant, L=4.0, n=4001, imp=0.30,
                 band_halfwidth=2.0):
        self.mp = mp
        self.variant = variant
        self.L = L
        self.lc = np.sqrt(2.0 * mp[2])
        self.x = np.linspace(0.0, L, n)
        self.h = self.x[1] - self.x[0]
        epsD = mp[3]
        epsD_x = np.full(n, epsD)
        c = 0.5 * L
        epsD_x[np.abs(self.x - c) <= 1.5 * self.h] *= (1.0 - imp)
        self.epsD_x = epsD_x
        self.kappa = np.zeros(n)
        c0 = mp[2] / self.h ** 2
        self.ab = np.zeros((3, n))
        self.ab[1, :] = 1.0 + 2.0 * c0
        self.ab[0, 1:] = -c0
        self.ab[2, :-1] = -c0
        self.ab[1, 0] = 1.0 + c0
        self.ab[1, -1] = 1.0 + c0
        self.band = (np.abs(self.x - c) <= band_halfwidth * self.lc)
        self.xb = self.x[self.band]

    def step(self, w_target, iters=400, tol=1e-10):
        kappa_c = self.kappa.copy()
        d_prev = _damage_vec(kappa_c, self.epsD_x, self.mp, self.variant)
        kap = kappa_c.copy()
        theta = 1.0
        err_prev = np.inf
        sig = 0.0
        for _ in range(iters):
            s = _secant_profile(kap, d_prev, self.epsD_x, self.mp,
                                self.variant, EBAR)
            phi = solve_banded((1, 1), self.ab, s)
            wb = float(np.trapezoid(s[self.band], self.xb))
            sig = w_target / max(wb, 1e-30)
            ebar = sig * phi
            kap_new = np.maximum(kappa_c, np.minimum(ebar, 1e3))
            err = (float(np.max(np.abs(kap_new - kap)))
                   / max(float(np.max(kap)), 1e-12))
            if err < tol:
                kap = kap_new
                break
            if err > err_prev and theta > 0.05:
                theta *= 0.5
            err_prev = err
            kap = np.maximum(kappa_c, kap + theta * (kap_new - kap))
        self.kappa = kap
        s = _secant_profile(kap, d_prev, self.epsD_x, self.mp,
                            self.variant, EBAR)
        delta = sig * float(np.trapezoid(s, self.x))
        return sig, delta


def measure_Gf(mp, variant, **kw):
    """Adaptive band-opening sweep of a weak-core bar.

    The central imperfection (epsD*0.7) nucleates the band while the far
    field stays elastic (with a vanishing imperfection the far field sits
    exactly at epsD at peak and the whole-bar dissipation O(L*D(epsD))
    contaminates the measurement).  Steps are subdivided if loading
    spreads beyond 8*lc (the intra-step 'melt race' artifact).  Gf is the
    exact dissipation integral int_x D(kappa_f(x)) dx, D = int Y dd dkappa.
    """
    bar = Bar1D(mp, variant, **kw)
    epsD = mp[3]
    lc = np.sqrt(2.0 * mp[2])
    w0 = 4.0 * lc * epsD
    d_cap = 1.0 if variant != K.VAR_CGD else 0.999 * mp[8]
    c = 0.5 * bar.L
    ci = int(np.argmin(np.abs(bar.x - c)))
    ws, sigs, deltas = [], [], []
    clean = bar.kappa.copy()
    w = 0.5 * w0 * 0.7          # elastic range of the weak core
    dw = 0.05 * w0
    bad = 0
    n_sub = 0
    while w < 80.0 * w0:
        bar.kappa = clean.copy()
        sig, delta = bar.step(w)
        d_c = _damage_vec(bar.kappa[ci:ci + 2], bar.epsD_x[ci:ci + 2],
                          mp, variant).max()
        pol = ((bar.kappa > 1.0001 * epsD)
               & (np.abs(bar.x - c) > 8.0 * lc)).any()
        if pol:
            bad += 1
            n_sub += 1
            if bad > 50 or dw < 1e-6 * w0:
                pass                      # accept and move on
            else:
                dw *= 0.5
                continue
        bad = 0
        clean = bar.kappa.copy()
        ws.append(w)
        sigs.append(sig)
        deltas.append(delta)
        if d_c >= 0.9995 * d_cap:
            break                          # full separation reached
        if variant == K.VAR_CGD and len(sigs) > 6 \
                and sig > 0.999 * sig_prev \
                and sig < 0.45 * max(sigs):
            break                          # residual plateau (no separation)
        sig_prev = sig
        dw = min(dw * 1.5, 0.15 * w0)
        w += dw
    sigs = np.array(sigs)
    deltas = np.array(deltas)
    sig_peak = float(sigs.max()) if len(sigs) else 0.0
    # dissipation energy from the final kappa profile
    kmax = float(bar.kappa.max())
    kg = np.linspace(epsD, max(kmax, epsD * 1.0001), 20000)
    W = _diss_density(kg, mp, variant, EBAR)
    D = np.concatenate(([0.0], np.cumsum(0.5 * (W[1:] + W[:-1])
                                         * np.diff(kg))))
    kap_c = np.clip(bar.kappa, epsD, kg[-1])
    Dk = np.interp(kap_c, kg, D)
    Gf = float(np.trapezoid(Dk, bar.x))
    d_end = _damage_vec(bar.kappa, bar.epsD_x, mp, variant)
    return Gf, sig_peak, sigs[-1], (np.array(ws), sigs, deltas, bar, d_end)


def calibrate_lc(name, variant, epsD, Gf_target, lo=0.004, hi=0.30,
                 tol=0.01):
    """bisection on the nonlocal length to match the fracture energy at
    the paper's transition shape (s1, s2)."""
    def f(lc):
        G, _, _, _ = measure_Gf(mp_array(name, epsD, lc=lc), variant)
        return G - Gf_target
    fa = f(lo)
    fb = f(hi)
    if fa * fb > 0:
        raise RuntimeError(f'lc bracket failed: G({lo})={fa+Gf_target:.3g}'
                           f' G({hi})={fb+Gf_target:.3g} '
                           f'target {Gf_target:.3g}')
    a, b = lo, hi
    for _ in range(18):
        m = 0.5 * (a + b)
        fm = f(m)
        if abs(fm) < tol * Gf_target:
            return m
        if fa * fm <= 0:
            b = m
        else:
            a, fa = m, fm
    return 0.5 * (a + b)


def run_variant(name, lc=None):
    variant = VARIANTS[name]
    print(f"\n=== variant {name} ===")
    epsD = solve_epsD(name, variant)
    mp0 = mp_array(name, epsD)
    sp, kp = peak_hom(mp0, variant)
    print(f"  epsD = {epsD:.6e}  ->  sigma_peak = {sp*1000:.1f} MPa"
          f" @ kappa = {kp:.4e}   (target {SIGC*1000:.1f} MPa)")
    if name == 'cgd':
        # Gf is unbounded (sigma -> (1-alpha)*sigma_c plateau): take the
        # MNLD length for band-width parity; report the cutoff value.
        lc = lc if lc is not None else 0.04
        note = '  [CGD: Gf unbounded; lc set for band parity]'
    else:
        lc = calibrate_lc(name, variant, epsD, GC)
        note = ''
    mp = mp_array(name, epsD, lc=lc)
    G, spk, send, (ws, sigs, deltas, bar, d_end) = measure_Gf(mp, variant)
    print(f"  lc   = {lc:.4f}  ->  Gf = {G/GC*100:.2f}% of gc"
          f"   bar sigma_peak = {spk*1000:.1f} MPa"
          f"   sigma_end = {send*1000:.2f} MPa{note}")
    bandz = d_end > 0.05
    width = float(bar.x[bandz].max() - bar.x[bandz].min()) if bandz.any() \
        else 0.0
    print(f"  band width (d>0.05) = {width:.4f} mm   d_max = {d_end.max():.4f}")
    # plots (sigma in MPa)
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.6))
    k_hi = epsD * (SHAPES[name]['s1'] * SHAPES[name]['s2'] * 1.05
                   if name != 'cgd' else 30.0)
    ks = np.linspace(epsD, k_hi, 800)
    axs[0].plot(ks / epsD, _sigma_hom(ks, mp, variant, EBAR) * 1000, 'b-')
    axs[0].axhline(SIGC * 1000, color='r', ls='--', lw=1,
                   label='AT2 sigma_c')
    axs[0].set_xlabel('kappa/epsD')
    axs[0].set_ylabel('sigma [MPa]')
    axs[0].set_title(f'{name}: homogeneous')
    axs[0].legend()
    axs[0].grid(alpha=0.3)
    axs[1].plot(deltas, sigs * 1000, 'b-')
    axs[1].set_xlabel('bar elongation [mm]')
    axs[1].set_ylabel('sigma [MPa]')
    axs[1].set_title(f'{name}: Gf={G*1000:.3f} N/mm')
    axs[1].grid(alpha=0.3)
    axs[2].semilogy(bar.x, np.maximum(d_end, 1e-6), 'b-')
    axs[2].set_xlabel('x [mm]')
    axs[2].set_ylabel('d')
    axs[2].set_title(f'{name}: band (lc={lc:.4f})')
    axs[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'results/calib1d_{name}.png', dpi=130)
    plt.close(fig)
    return dict(epsD=float(epsD), lc=float(lc), Gf=float(G),
                sigma_peak=float(sp * 1000), band_width=width,
                kappa_peak=float(kp), beta=float(mp[5]),
                sigma_end=float(send * 1000))


if __name__ == '__main__':
    print(f"[calib] plane_stress={PLANE_STRESS}  E_eff={EBAR:.2f} kN/mm^2  "
          f"sigma_c={SIGC*1000:.1f} MPa  Gf target={GC} kN/mm")
    os.makedirs('results', exist_ok=True)
    names = sys.argv[1:] or list(VARIANTS)
    order = [nm for nm in names if nm != 'cgd'] + \
            [nm for nm in names if nm == 'cgd']
    out = {}
    lc_ref = None
    for nm in order:
        if nm == 'cgd' and lc_ref is not None:
            out[nm] = run_variant(nm, lc=lc_ref)
        else:
            out[nm] = run_variant(nm)
            if nm == 'mnld':
                lc_ref = out[nm]['lc']
    with open('results/calib1d.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(f'\n// calibrated parameters (Jin material, '
          f'{"plane stress" if PLANE_STRESS else "plane strain"}):')
    for nm, r in out.items():
        print(f"//   {nm}: epsD={r['epsD']:.4e}, beta={r['beta']:.2f}, "
              f"lc={r['lc']:.4f}  (Gf={r['Gf']/GC*100:.1f}% gc, "
              f"band={r['band_width']:.3f} mm, sig_end={r['sigma_end']:.1f}"
              f" MPa)")
