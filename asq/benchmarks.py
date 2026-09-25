"""
Benchmark case definitions.

Sources:
  Jin 2024 (10.1002/nme.7572): SNT / SNS / TPB phase-field benchmarks.
  MNLD paper (arXiv 2506.24099) + author code: SNS-100, TPB-2000, L-panel.
  UAL paper: SNS-100 with CGD (original nonlocal damage baseline).
"""
import numpy as np
from . import quadtree as qt
from . import kernels
from .kernels import VAR_CGD, VAR_MNLD, VAR_SC

# ----------------------------------------------------------------------
#  Jin 2024 material (plane strain): lam=121.15, mu=80.77 kN/mm^2,
#  gc = 2.7e-3 kN/mm, l = 0.0075 mm
# ----------------------------------------------------------------------
JIN_LAM = 121.15
JIN_MU = 80.77
JIN_GC = 2.7e-3
JIN_L = 0.0075
JIN_EP = JIN_LAM + 2 * JIN_MU          # 282.69 GPa plane-strain modulus
# AT2 plane-strain strength (2D), this code's convention w = gc d^2/(2l):
#   sigma_c = 0.3247*sqrt(E'*Gc/l)  (nucleation of the homogeneous branch)
JIN_SIGC = 0.3247 * np.sqrt(JIN_EP * JIN_GC / JIN_L)     # ~103.6 MPa
JIN_EPSC = JIN_SIGC / JIN_EP                             # ~3.665e-4

# Calibrated gradient-damage parameters on the Jin material, PLANE STRESS
# (calib1d.py): epsD from the AT2 homogeneous strength
# sigma_c = 0.3247*sqrt(E*gc/l) = 2823 MPa (epsD ~ eps_c = 1.344e-2);
# lc from the 1D localized fracture energy Gf = gc at the paper's
# transition shape (s1=1.5, s2=9, alpha=0.96, beta*epsD = 4 for a peak at
# onset).  CGD cannot match a finite Gf (residual plateau (1-alpha)*sc):
# its lc is set to the MNLD value for band-width parity.
JIN_GD_COMMON = dict(s1=1.5, s2=9.0, dmax=0.999, n_fr=100.0, ST=1e-7,
                     eq_type=kernels.EQ_MAZARS, law=kernels.LAW_MAZARS,
                     k0=10.0, nu=0.2)
JIN_GD_VARIANTS = {
    'cgd': dict(JIN_GD_COMMON, epsD=1.344382e-2, alpha=0.7,
                beta_law=148.77, lc=0.0323, kappa_s=1e-6),
    'mnld': dict(JIN_GD_COMMON, epsD=1.344382e-2, alpha=0.96,
                 beta_law=297.53, lc=0.0323, kappa_s=1e-8),
    'sc': dict(JIN_GD_COMMON, epsD=1.343633e-2, alpha=0.96,
               beta_law=297.70, lc=0.0122, kappa_s=1e-6,
               a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0),
}
JIN_GD = JIN_GD_VARIANTS['mnld']   # backwards-compatible default


def _jin_gd_mat(variant):
    key = {0: 'cgd', 1: 'mnld', 2: 'sc'}.get(int(variant), 'mnld')
    m = dict(JIN_GD_VARIANTS[key])
    m['lam'] = JIN_LAM
    m['mu'] = JIN_MU
    return m


def jin_snt_pf(**over):
    c = dict(
        model='pf',
        x0=0, y0=0, x1=1, y1=1, nx=2, ny=2, max_depth=12,
        slits=[qt.Slit((0, 0.5), (0.5, 0.5))],
        h_min=0.5 / 2 ** 8, h_max=0.5, grade=2.0, r_buf=JIN_L,
        d_thr=0.2, psi_frac=0.5,
        init_features=[dict(pts=[(0.5, 0.5)], grade=1.0, r=4 * JIN_L)],
        fix=[(('edge', 0, 0, 1, 0), 'y'), (('point', 0, 0), 'x')],
        load=[(('edge', 0, 1, 1, 1), 'y', 1.0)],
        mat=dict(lam=JIN_LAM, mu=JIN_MU, gc=JIN_GC, l=JIN_L),
        lam_end=0.008, u_scale=1.0, dlam0=0.02 * 0.008,
        tau0=5e-5, dl0=1e-4, max_steps=600, onset_d=0.05,
        plane_stress=True, f_end_frac=0.30, wall_max_s=2100,
        name='jin_snt_pf')
    c.update(over)
    return c


def jin_sns_pf(**over):
    c = dict(
        model='pf',
        x0=0, y0=0, x1=1, y1=1, nx=2, ny=2, max_depth=12,
        slits=[qt.Slit((0, 0.5), (0.5, 0.5))],
        h_min=0.5 / 2 ** 7, h_max=0.5, grade=2.0, r_buf=JIN_L,
        d_thr=0.2, psi_frac=0.5,
        init_features=[dict(pts=[(0.5, 0.5)], grade=1.0, r=4 * JIN_L)],
        fix=[(('edge', 0, 0, 1, 0), 'x'), (('edge', 0, 0, 1, 0), 'y'),
             (('edge', 0, 1, 1, 1), 'y'),
             (('edge', 0, 0, 0, 1), 'y'), (('edge', 1, 0, 1, 1), 'y')],
        load=[(('edge', 0, 1, 1, 1), 'x', 1.0)],
        mat=dict(lam=JIN_LAM, mu=JIN_MU, gc=JIN_GC, l=JIN_L),
        lam_end=0.016, u_scale=1.0, dlam0=0.02 * 0.016,
        tau0=5e-5, dl0=1e-4, max_steps=600, onset_d=0.05,
        plane_stress=True, f_end_frac=0.25, name='jin_sns_pf')
    c.update(over)
    return c


def jin_tpb_pf(**over):
    c = dict(
        model='pf',
        x0=0, y0=0, x1=8, y1=2, nx=40, ny=10, max_depth=12,
        slits=[qt.Slit((4, 0), (4, 0.4))],
        h_min=0.2 / 2 ** 6, h_max=0.2, grade=2.0, r_buf=JIN_L,
        d_thr=0.2, psi_frac=0.5,
        init_features=[dict(pts=[(4, 0.4)], grade=1.0, r=4 * JIN_L)],
        fix=[(('point', 0, 0), 'x'), (('point', 0, 0), 'y'),
             (('point', 8, 0), 'y')],
        load=[(('point', 4, 2), 'y', -1.0)],
        mat=dict(lam=12.0, mu=8.0, gc=5e-4, l=JIN_L),
        lam_end=0.062, u_scale=1.0, dlam0=0.02 * 0.062,
        tau0=5e-5, dl0=1e-4, max_steps=600, onset_d=0.05, f_end_frac=0.10,
        wall_max_s=7200, name='jin_tpb_pf')
    c.update(over)
    return c


def jin_snt_gd(variant, mat_over=None, **over):
    m = _jin_gd_mat(variant)
    if mat_over:
        m.update(mat_over)
    hmin = 0.5 / 2 ** (7 if variant == VAR_SC else 5)
    c = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=1, y1=1, nx=2, ny=2, max_depth=12,
        slits=[qt.Slit((0, 0.5), (0.5, 0.5))],
        h_min=hmin, h_max=0.5, grade=1.5, r_buf=m['lc'] / 2,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(0.5, 0.5)], grade=1.0, r=m['lc'])],
        fix=[(('edge', 0, 0, 1, 0), 'y'), (('point', 0, 0), 'x')],
        load=[(('edge', 0, 1, 1, 1), 'y', 1.0)],
        mat=m,
        lam_end=0.012, u_scale=1.0, dlam0=0.02 * 0.012,
        tau0=2e-4, dl0=1e-4, max_steps=600,
        plane_stress=True, name=f"jin_snt_gd_v{variant}")
    c.update(over)
    return c


def jin_sns_gd(variant, mat_over=None, **over):
    m = _jin_gd_mat(variant)
    if mat_over:
        m.update(mat_over)
    hmin = 0.5 / 2 ** (7 if variant == VAR_SC else 5)
    c = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=1, y1=1, nx=2, ny=2, max_depth=12,
        slits=[qt.Slit((0, 0.5), (0.5, 0.5))],
        h_min=hmin, h_max=0.5, grade=1.5, r_buf=m['lc'] / 2,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(0.5, 0.5)], grade=1.0, r=m['lc'])],
        fix=[(('edge', 0, 0, 1, 0), 'x'), (('edge', 0, 0, 1, 0), 'y'),
             (('edge', 0, 1, 1, 1), 'y'),
             (('edge', 0, 0, 0, 1), 'y'), (('edge', 1, 0, 1, 1), 'y')],
        load=[(('edge', 0, 1, 1, 1), 'x', 1.0)],
        mat=m,
        lam_end=0.020, u_scale=1.0, dlam0=0.02 * 0.020,
        tau0=2e-4, dl0=1e-4, max_steps=600,
        plane_stress=True, name=f"jin_sns_gd_v{variant}")
    c.update(over)
    return c


# ----------------------------------------------------------------------
#  MNLD-paper benchmarks (SNS-100, TPB-2000, L-panel)
# ----------------------------------------------------------------------
def sns100(variant, **over):
    m = dict(lam=2 * 125.0 * 0.2 / 0.6, mu=125.0, lc=2.0, epsD=1e-4,
             alpha=0.7, beta_law=1e4, s1=1.5, s2=5.0, dmax=0.999,
             n_fr=100.0, ST=1e-7, eq_type=kernels.EQ_VM, k0=10.0, nu=0.2,
             law=kernels.LAW_MAZARS,
             a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0,
             kappa_s=1e-8 if variant == VAR_MNLD else 1e-6)
    c = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=100, y1=100, nx=2, ny=2, max_depth=10,
        slits=[qt.Slit((0, 50), (50, 50))],
        h_min=50 / 2 ** 6, h_max=50, grade=1.0, r_buf=1.0,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(50, 50)], grade=1.0, r=1.0)],
        fix=[(('edge', 0, 0, 100, 0), 'x'),
             (('edge', 0, 0, 0, 100), 'y'),
             (('edge', 100, 0, 100, 100), 'y')],
        load=[(('edge', 0, 100, 100, 100), 'x', 0.019)],
        mat=m,
        lam_end=1.05, u_scale=0.019, dlam0=0.02,
        tau0=2e-4, dl0=2e-3, max_steps=500,
        name=f"sns100_v{variant}")
    c.update(over)
    return c


def tpb2000(variant, **over):
    m = dict(lam=2 * 8333.4 * 0.2 / 0.6, mu=8333.4, lc=6.0, epsD=1e-4,
             alpha=0.99, beta_law=1500.0, s1=1.5, s2=8.0, dmax=0.999,
             n_fr=100.0, ST=1e-7, eq_type=kernels.EQ_MAZARS,
             law=kernels.LAW_GEERS, k0=10.0, nu=0.2,
             a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0,
             kappa_s=1e-8 if variant == VAR_MNLD else 1e-6)
    c = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=2000, y1=300, nx=200, ny=30, max_depth=10,
        slits=[qt.Slit((1000, 190), (1000, 300))],
        h_min=10 / 2 ** 2, h_max=10, grade=1.0, r_buf=3.0,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(1000, 190)], grade=1.0, r=3.0)],
        fix=[(('point', 0, 300), 'x'), (('point', 0, 300), 'y'),
             (('point', 2000, 300), 'y')],
        load=[(('point', 1000, 0), 'y', 1.0)],
        mat=m,
        lam_end=1.0, u_scale=1.0, dlam0=0.02,
        tau0=2e-3, dl0=5e-3, max_steps=500,
        name=f"tpb2000_v{variant}")
    c.update(over)
    return c


def lshape(variant, dlam_max=None, **over):
    m = dict(lam=2 * 8000.0 * 0.18 / 0.64, mu=8000.0, lc=6.0, epsD=2.5e-4,
             alpha=0.96, beta_law=600.0, s1=1.5, s2=9.0, dmax=0.999,
             n_fr=100.0, ST=1e-7, eq_type=kernels.EQ_VM, k0=10.0, nu=0.18,
             law=kernels.LAW_GEERS,
             a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0,
             kappa_s=1e-8 if variant == VAR_MNLD else 1e-6)

    def lmask(a, b, c_, d_):
        return not (c_ > 250 + 1e-9 and b < 250 - 1e-9)

    cc = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=500, y1=500, nx=2, ny=2, max_depth=10,
        mask=lmask,
        h_min=250 / 2 ** 7, h_max=250, grade=1.0, r_buf=3.0,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(250, 250), (485, 250)], grade=1.0, r=3.0),
                       dict(pts=[(485, 251)], grade=1.0, r=45.0)],
        fix=[(('edge', 0, 0, 250, 0), 'x'), (('edge', 0, 0, 250, 0), 'y')],
        load=[(('edge', 470, 250, 500, 250), 'y', 1.0)],
        mat=m,
        lam_end=1.0, u_scale=1.0, dlam0=0.02,
        tau0=2e-3, dl0=5e-3, max_steps=500,
        name=f"lshape_v{variant}")
    if dlam_max is not None:
        # emulate the paper's load-step-limit study via tau caps
        cc['tau_max'] = dlam_max * 2e-3
        cc['name'] += f"_dl{dlam_max}"
    cc.update(over)
    return cc


# ----------------------------------------------------------------------
#  smoke-test mini cases
# ----------------------------------------------------------------------
def mini_sns(variant=VAR_SC, **over):
    m = dict(lam=2 * 125.0 * 0.2 / 0.6, mu=125.0, lc=2.0, epsD=1e-4,
             alpha=0.7, beta_law=1e4, s1=1.5, s2=5.0, dmax=0.999,
             n_fr=100.0, ST=1e-7, eq_type=kernels.EQ_VM, k0=10.0, nu=0.2,
             law=kernels.LAW_MAZARS,
             a_sc=0.8, p_sc=1.0, s_sc=2.0, m_sc=1000.0, kappa_s=1e-6)
    c = dict(
        model='gd', variant=variant,
        x0=0, y0=0, x1=20, y1=20, nx=2, ny=2, max_depth=8,
        slits=[qt.Slit((0, 10), (10, 10))],
        h_min=10 / 2 ** 5, h_max=10, grade=1.0, r_buf=1.0,
        theta=0.9, d_thr=1e-3,
        init_features=[dict(pts=[(10, 10)], grade=1.0, r=1.0)],
        fix=[(('edge', 0, 0, 20, 0), 'x'),
             (('edge', 0, 0, 0, 20), 'y'),
             (('edge', 20, 0, 20, 20), 'y')],
        load=[(('edge', 0, 20, 20, 20), 'x', 0.019)],
        mat=m,
        lam_end=1.1, u_scale=0.019, dlam0=0.02,
        tau0=2e-4, dl0=2e-3, max_steps=300,
        name=f"mini_sns_v{variant}")
    c.update(over)
    return c


def mini_snt_pf(**over):
    c = dict(
        model='pf',
        x0=0, y0=0, x1=1, y1=1, nx=2, ny=2, max_depth=12,
        slits=[qt.Slit((0, 0.5), (0.5, 0.5))],
        h_min=0.5 / 2 ** 6, h_max=0.5, grade=2.0, r_buf=JIN_L,
        d_thr=0.2, psi_frac=0.5,
        init_features=[dict(pts=[(0.5, 0.5)], grade=1.0, r=4 * JIN_L)],
        fix=[(('edge', 0, 0, 1, 0), 'y'), (('point', 0, 0), 'x')],
        load=[(('edge', 0, 1, 1, 1), 'y', 1.0)],
        mat=dict(lam=JIN_LAM, mu=JIN_MU, gc=2.7e-2, l=0.03),
        lam_end=0.02, u_scale=1.0, dlam0=0.02 * 0.02,
        tau0=1e-3, dl0=1e-3, max_steps=200, onset_d=0.05,
        name='mini_snt_pf')
    c.update(over)
    return c
