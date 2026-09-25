"""
Numba constitutive + assembly kernels for
  (a) gradient-enhanced damage variants (CGD / MNLD / SC-MNLD)
  (b) AT2 phase-field (Miehe spectral split)

DoF layout (per mesh):  ux=2i, uy=2i+1  (i = vertex),  scalar (ebar or d) = 2n+i
Element local layout:   u dofs 0..2m-1 (2j, 2j+1), scalar dofs 2m..2m+m-1

Voigt strain (exx, eyy, gxy);  stress (sxx, syy, sxy) with sxy = mu*gxy.
B rows: [0]=dN/dx (exx), [1]=dN/dy (eyy), [2]= shear: B[2,2j]=dN/dy, B[2,2j+1]=dN/dx.
Hence for any vectors:  r_u[a] = sum_r B[r,a]*sig[r]  (no parity branches).
"""
import numpy as np
from numba import njit, prange

MAXV = 8
LMAX = 3 * MAXV  # 24 local dofs

# ---------------- variant ids ----------------
VAR_CGD = 0        # conventional gradient damage (original nonlocal / UAL)
VAR_MNLD = 1       # paper MNLD: f_a = d^2/2 - d_prev^2/2, 0.8-power transition
VAR_SC = 2         # SC-MNLD: state function g*, C1 Hermite transition

EQ_MAZARS = 0
EQ_VM = 1          # modified von Mises (de Vree) with (k0, nu)
LAW_MAZARS = 0
LAW_GEERS = 1

# mp layout (gradient damage):
#  0 lam  1 mu  2 c (=lc^2/2)  3 epsD  4 alpha  5 beta  6 s1  7 s2
#  8 dmax  9 n_fr  10 ST  11 variant  12 eq_type  13 law  14 k0  15 nu
# 16 a_sc 17 p_sc 18 s_sc 19 m_sc 20 kappa_s


@njit(cache=True, inline='always')
def eq_strain(exx, eyy, gxy, eq_type, k0, nu):
    """local equivalent strain + derivatives wrt (exx,eyy,gxy)."""
    if eq_type == EQ_MAZARS:
        mm = 0.5 * (exx + eyy)
        R = np.sqrt((0.5 * (exx - eyy)) ** 2 + (0.5 * gxy) ** 2)
        l1 = mm + R
        l2 = mm - R
        p1 = l1 if l1 > 0.0 else 0.0
        p2 = l2 if l2 > 0.0 else 0.0
        eq = np.sqrt(p1 * p1 + p2 * p2)
        if eq > 1e-14:
            if R > 1e-14:
                dl1x = 0.5 + 0.25 * (exx - eyy) / R
                dl1y = 0.5 - 0.25 * (exx - eyy) / R
                dl1g = 0.25 * gxy / R
                dl2x = 1.0 - dl1x
                dl2y = 1.0 - dl1y
                dl2g = -dl1g
            else:
                dl1x, dl1y, dl1g = 1.0, 0.0, 0.0
                dl2x, dl2y, dl2g = 0.0, 1.0, 0.0
            dex = (p1 * dl1x + p2 * dl2x) / eq
            dey = (p1 * dl1y + p2 * dl2y) / eq
            deg = (p1 * dl1g + p2 * dl2g) / eq
            return eq, dex, dey, deg
        return 0.0, 0.0, 0.0, 0.0
    else:
        # modified von Mises (de Vree), plane-strain invariants
        I1 = exx + eyy
        dlt = exx - eyy
        c1 = (k0 - 1.0) / (2.0 * k0 * (1.0 - 2.0 * nu))
        A = ((k0 - 1.0) / (1.0 - 2.0 * nu)) ** 2 + k0 / (1.0 + nu) ** 2
        B = 12.0 * k0 / (1.0 + nu) ** 2
        S = A * I1 * I1 + 0.25 * B * (dlt * dlt + gxy * gxy)
        R = np.sqrt(S) if S > 0.0 else 0.0
        eq = c1 * I1 + R / (2.0 * k0)
        if R > 1e-16:
            f = 0.25 / (k0 * R)
            dSx = 2.0 * A * I1 + 0.5 * B * dlt
            dSy = 2.0 * A * I1 - 0.5 * B * dlt
            dSg = 0.5 * B * gxy
            dex = c1 + dSx * f
            dey = c1 + dSy * f
            deg = dSg * f
        else:
            dex, dey, deg = c1, c1, 0.0
        if eq < 0.0:
            eq = 0.0
            dex = dey = deg = 0.0
        return eq, dex, dey, deg


@njit(cache=True, inline='always')
def base_law(kappa, epsD, alpha, beta, law):
    """raw Mazars/Geers damage + derivative (no clipping)."""
    if kappa <= epsD:
        return 0.0, 0.0
    if law == LAW_MAZARS:
        e = np.exp(-beta * (kappa - epsD))
        d = 1.0 - epsD * (1.0 - alpha) / kappa - alpha * e
        dd = epsD * (1.0 - alpha) / (kappa * kappa) + alpha * beta * e
        return d, dd
    else:
        e = np.exp(beta * (epsD - kappa))
        t = (1.0 - alpha) + alpha * e
        d = 1.0 - (epsD / kappa) * t
        dd = (epsD / (kappa * kappa)) * t + (epsD / kappa) * alpha * beta * e
        return d, dd


@njit(cache=True, inline='always')
def damage_law(kappa, mp, variant):
    """damage + d(damage)/d(kappa)."""
    epsD = mp[3]
    alpha = mp[4]
    beta = mp[5]
    s1 = mp[6]
    s2 = mp[7]
    dmax = mp[8]
    law = mp[13]
    if kappa <= epsD:
        return 0.0, 0.0
    if variant == VAR_CGD:
        d, dd = base_law(kappa, epsD, alpha, beta, law)
        d = d * dmax
        if d < 0.0:
            d = 0.0
            dd = 0.0
        if d >= dmax:
            d = dmax
            dd = 0.0
        else:
            dd = dd * dmax
        return d, dd
    k_trans = epsD * s2
    k_final = s1 * k_trans
    if kappa <= k_trans:
        d, dd = base_law(kappa, epsD, alpha, beta, law)
        if d < 0.0:
            d = 0.0
            dd = 0.0
        if d > 1.0:
            d = 1.0
            dd = 0.0
        d = d * dmax
        dd = dd * dmax
        if d >= dmax:
            d = dmax
            dd = 0.0
        return d, dd
    if kappa >= k_final:
        return 1.0, 0.0
    dtr_raw, ddtr = base_law(k_trans, epsD, alpha, beta, law)
    if variant == VAR_MNLD:
        D_trans = dtr_raw            # deal.II convention (raw)
        if D_trans < 0.0:
            D_trans = 0.0
        if D_trans > 1.0:
            D_trans = 1.0
        dk = k_final - k_trans
        xi = (kappa - k_trans) / dk
        d = D_trans + (1.0 - D_trans) * xi ** 0.8
        dd = (0.8 * (1.0 - D_trans) / dk) * xi ** (-0.2)
        if d > 1.0:
            d = 1.0
            dd = 0.0
        return d, dd
    else:
        D_trans = dtr_raw * dmax     # continuous with base branch
        if D_trans < 0.0:
            D_trans = 0.0
        if D_trans > 1.0:
            D_trans = 1.0
        dk = k_final - k_trans
        sh = ddtr * dmax * dk / max(1.0 - D_trans, 1e-12)
        if sh > 3.0:
            sh = 3.0
        xi = (kappa - k_trans) / dk
        H = sh * xi + (3.0 - 2.0 * sh) * xi * xi + (sh - 2.0) * xi ** 3
        Hp = sh + 2.0 * (3.0 - 2.0 * sh) * xi + 3.0 * (sh - 2.0) * xi * xi
        d = D_trans + (1.0 - D_trans) * H
        dd = (1.0 - D_trans) * Hp / dk
        if d > 1.0:
            d = 1.0
            dd = 0.0
        if d < 0.0:
            d = 0.0
            dd = 0.0
        return d, dd


@njit(cache=True, inline='always')
def degradation(d, d_prev, mp, variant):
    """effective stress degradation g_eff and dg_eff/dd (current d)."""
    if variant == VAR_CGD:
        return 1.0 - d, -1.0
    if variant == VAR_MNLD:
        ks = mp[20] if mp[20] > 0.0 else 1e-9
        g = (1.0 - d) + 0.5 * d * d - 0.5 * d_prev * d_prev
        if g < ks:            # numerical floor (fully cracked cells)
            return ks, 0.0
        return g, -1.0 + d
    a = mp[16]
    p = mp[17]
    s = mp[18]
    mm = mp[19]
    ks = mp[20]
    phi = a * d ** p * (1.0 - d) ** s
    dphi = a * (p * d ** (p - 1.0) * (1.0 - d) ** s
                - s * d ** p * (1.0 - d) ** (s - 1.0))
    h = 1.0 - d + phi
    q = 1.0 - d ** mm
    dq = -mm * d ** (mm - 1.0)
    return h * q + ks, (-1.0 + dphi) * q + h * dq


@njit(cache=True)
def assemble_gd(elem_ptr, elem_n, elem_flat, nnode,
                sB, sT, sG, sM, sD, sA, sub_off,
                U, kappa_prev, d_prev_inc, frozen,
                mp, C,
                vals_out, rvals_out,
                kappa_out, d_out, Y_out, psi_out,
                want_tangent):
    """Assemble residual (+tangent) for gradient-damage variants."""
    ne = len(elem_ptr) - 1
    ST = mp[10]
    variant = mp[11]
    eq_type = mp[12]
    k0 = mp[14]
    nu = mp[15]
    c = mp[2]
    n_fr = mp[9]

    for e in prange(ne):
        m = elem_n[e]
        vids = elem_flat[elem_ptr[e]:elem_ptr[e] + m]
        ue = np.zeros(2 * MAXV)
        eb = np.zeros(MAXV)
        for j in range(m):
            ue[2 * j] = U[2 * vids[j]]
            ue[2 * j + 1] = U[2 * vids[j] + 1]
            eb[j] = U[2 * nnode + vids[j]]
        Kloc = np.zeros((LMAX, LMAX))
        rloc = np.zeros(LMAX)
        ns = sub_off[e]
        nf = sub_off[e + 1]
        for si in range(ns, nf):
            A_k = sA[si]
            Bk = sB[si]
            Tk = sT[si]
            Gk = sG[si]
            Mk = sM[si]
            Dk = sD[si]
            exx = 0.0
            eyy = 0.0
            gxy = 0.0
            for c2 in range(2 * MAXV):
                exx += Bk[0, c2] * ue[c2]
                eyy += Bk[1, c2] * ue[c2]
                gxy += Bk[2, c2] * ue[c2]
            eq, dex, dey, deg = eq_strain(exx, eyy, gxy, eq_type, k0, nu)
            ebk = 0.0
            for j in range(m):
                ebk += Dk[j] * eb[j]
            kp = kappa_prev[si]
            if frozen or (kp - ebk > ST):
                loading = False
                kappa = kp
                d, dd = damage_law(kappa, mp, variant)
                dd = 0.0
            else:
                loading = True
                kappa = ebk
                d, dd = damage_law(kappa, mp, variant)
                if d <= 0.0 or d >= 1.0:
                    dd = 0.0
            kappa_out[si] = kappa
            d_out[si] = d
            dprev = d_prev_inc[si]
            g_eff, dg_dd = degradation(d, dprev, mp, variant)
            if variant == VAR_CGD:
                fr = 1.0
                dfr = 0.0
            else:
                fr = 1.0 - d ** n_fr
                dfr = -n_fr * d ** (n_fr - 1.0)
            sxx = g_eff * (C[0, 0] * exx + C[0, 1] * eyy)
            syy = g_eff * (C[1, 0] * exx + C[1, 1] * eyy)
            sxy = g_eff * C[2, 2] * gxy
            sig = np.array([sxx, syy, sxy])
            psi0 = 0.5 * (C[0, 0] * exx * exx + C[1, 1] * eyy * eyy
                          + 2.0 * C[0, 1] * exx * eyy + C[2, 2] * gxy * gxy)
            if variant == VAR_MNLD:
                Y_out[si] = (1.0 - d) * psi0
            else:
                Y_out[si] = -dg_dd * psi0
            psi_out[si] = psi0
            # r_u = A B^T sig
            for a in range(2 * m):
                rloc[a] += A_k * (Bk[0, a] * sig[0] + Bk[1, a] * sig[1]
                                  + Bk[2, a] * sig[2])
            # r_e = M ebar + c A G^T G ebar - (A/3) fr eq Tk
            g0 = 0.0
            g1 = 0.0
            for j in range(m):
                g0 += Gk[0, j] * eb[j]
                g1 += Gk[1, j] * eb[j]
            src = (A_k / 3.0) * fr * eq
            for i2 in range(m):
                acc = 0.0
                for j2 in range(m):
                    acc += Mk[i2, j2] * eb[j2]
                acc += c * A_k * (Gk[0, i2] * g0 + Gk[1, i2] * g1)
                acc -= src * Tk[i2]
                rloc[2 * m + i2] += acc
            if want_tangent:
                gC = np.empty((3, 3))
                for r in range(3):
                    for c2 in range(3):
                        gC[r, c2] = g_eff * C[r, c2]
                # J_uu = A B^T gC B  (full contraction)
                for a in range(2 * m):
                    for b in range(2 * m):
                        acc = 0.0
                        for r in range(3):
                            Ba = Bk[r, a]
                            if Ba != 0.0:
                                for c2 in range(3):
                                    acc += Ba * gC[r, c2] * Bk[c2, b]
                        Kloc[a, b] += A_k * acc
                # couplings
                if loading and dd != 0.0:
                    Cex = C[0, 0] * exx + C[0, 1] * eyy
                    Cey = C[1, 0] * exx + C[1, 1] * eyy
                    Ceg = C[2, 2] * gxy
                    fac = A_k * dg_dd * dd
                    for a in range(2 * m):
                        va = Bk[0, a] * Cex + Bk[1, a] * Cey + Bk[2, a] * Ceg
                        for j in range(m):
                            Kloc[a, 2 * m + j] += fac * va * Dk[j]
                sv0, sv1, sv2 = dex, dey, deg
                for i2 in range(m):
                    for b in range(2 * m):
                        vb = (sv0 * Bk[0, b] + sv1 * Bk[1, b]
                              + sv2 * Bk[2, b])
                        Kloc[2 * m + i2, b] += -(A_k / 3.0) * fr * Tk[i2] * vb
                for i2 in range(m):
                    for j2 in range(m):
                        Kloc[2 * m + i2, 2 * m + j2] += (
                            Mk[i2, j2]
                            + c * A_k * (Gk[0, i2] * Gk[0, j2]
                                         + Gk[1, i2] * Gk[1, j2]))
                if loading and dd != 0.0 and variant != VAR_CGD:
                    f2 = -(A_k / 3.0) * eq * dfr * dd
                    for i2 in range(m):
                        for j2 in range(m):
                            Kloc[2 * m + i2, 2 * m + j2] += f2 * Tk[i2] * Dk[j2]
        for a in range(LMAX):
            rvals_out[e, a] = rloc[a]
            if want_tangent:
                for b in range(LMAX):
                    vals_out[e, a, b] = Kloc[a, b]


# ----------------------------------------------------------------------
#  AT2 phase field, Miehe spectral split
# ----------------------------------------------------------------------
@njit(cache=True, inline='always')
def spectral(exx, eyy, gxy, lam, mu):
    """Miehe spectral split in 2D. Exact stresses AND exact tangent.

    Uses invariant coordinates m=(exx+eyy)/2, d=(exx-eyy)/2, b=gxy/2,
    R=sqrt(d^2+b^2), eigen strains l1=m+R, l2=m-R, c^2=(1+d/R)/2,
    s^2=(1-d/R)/2, cs=b/(2R):
      sp = lam<I1>+ + mu*(S1+S2) + mu*(S1-S2)*d/R   (xx),  -d/R (yy)
      sp_xy = mu*(S1-S2)*b/R
    with S1=<l1>+, S2=<l2>+ (Macaulay); sm analogous with negative parts.
    Returns (sp, sm, Cp, Cm, psip);  stress Voigt (sxx,syy,sxy), sxy=mu*gxy.
    """
    m = 0.5 * (exx + eyy)
    dv = 0.5 * (exx - eyy)
    b = 0.5 * gxy
    I1 = exx + eyy
    R = np.sqrt(dv * dv + b * b)
    l1 = m + R
    l2 = m - R
    S1 = l1 if l1 > 0.0 else 0.0
    S2 = l2 if l2 > 0.0 else 0.0
    T1 = l1 if l1 < 0.0 else 0.0
    T2 = l2 if l2 < 0.0 else 0.0
    I1p = I1 if I1 > 0.0 else 0.0
    I1m = I1 if I1 < 0.0 else 0.0
    sp = np.empty(3)
    sm = np.empty(3)
    Cp = np.zeros((3, 3))
    Cm = np.zeros((3, 3))
    if R > 1e-24:
        A = S1 + S2
        D = S1 - S2
        Am = T1 + T2
        Dm = T1 - T2
        t = dv / R
        u = b / R
        sp[0] = lam * I1p + mu * A + mu * D * t
        sp[1] = lam * I1p + mu * A - mu * D * t
        sp[2] = mu * D * u
        sm[0] = lam * I1m + mu * Am + mu * Dm * t
        sm[1] = lam * I1m + mu * Am - mu * Dm * t
        sm[2] = mu * Dm * u
        # derivative seeds wrt (exx, eyy, gxy)
        dm = (0.5, 0.5, 0.0)
        dd = (0.5, -0.5, 0.0)
        db = (0.0, 0.0, 0.5)
        for k in range(3):
            dR = (dv * dd[k] + b * db[k]) / R
            dl1 = dm[k] + dR
            dl2 = dm[k] - dR
            dS1 = dl1 if l1 > 0.0 else 0.0
            dS2 = dl2 if l2 > 0.0 else 0.0
            dT1 = dl1 if l1 < 0.0 else 0.0
            dT2 = dl2 if l2 < 0.0 else 0.0
            dI1p = 2.0 * dm[k] if I1 > 0.0 else 0.0
            dI1m = 2.0 * dm[k] if I1 < 0.0 else 0.0
            dA = dS1 + dS2
            dD = dS1 - dS2
            dAm = dT1 + dT2
            dDm = dT1 - dT2
            dt = (dd[k] - t * dR) / R
            du = (db[k] - u * dR) / R
            f = dD * t + D * dt
            fm = dDm * t + Dm * dt
            Cp[0, k] = lam * dI1p + mu * dA + mu * f
            Cp[1, k] = lam * dI1p + mu * dA - mu * f
            Cp[2, k] = mu * (dD * u + D * du)
            Cm[0, k] = lam * dI1m + mu * dAm + mu * fm
            Cm[1, k] = lam * dI1m + mu * dAm - mu * fm
            Cm[2, k] = mu * (dDm * u + Dm * du)
    else:
        # isotropic point (l1=l2=m): directional limit of the tangent is the
        # full elastic tensor on the active side; at m==0 the limit is
        # direction-dependent -> use the symmetric average (keeps J
        # non-singular; the residual itself is exact)
        sp = np.zeros(3)
        sm = np.zeros(3)
        if m > 0.0:
            sp[0] = lam * I1p + 2.0 * mu * m
            sp[1] = sp[0]
            Cp[0, 0] = lam + 2.0 * mu
            Cp[1, 1] = lam + 2.0 * mu
            Cp[0, 1] = lam
            Cp[1, 0] = lam
            Cp[2, 2] = mu
        elif m < 0.0:
            sm[0] = lam * I1m + 2.0 * mu * m
            sm[1] = sm[0]
            Cm[0, 0] = lam + 2.0 * mu
            Cm[1, 1] = lam + 2.0 * mu
            Cm[0, 1] = lam
            Cm[1, 0] = lam
            Cm[2, 2] = mu
        else:
            h = 0.5 * (lam + 2.0 * mu)
            Cp[0, 0] = h
            Cp[1, 1] = h
            Cp[0, 1] = 0.5 * lam
            Cp[1, 0] = 0.5 * lam
            Cp[2, 2] = 0.5 * mu
            Cm[:] = Cp
    psip = 0.5 * lam * I1p * I1p + mu * (S1 * S1 + S2 * S2)
    return sp, sm, Cp, Cm, psip


@njit(cache=True)
def assemble_pf(elem_ptr, elem_n, elem_flat, nnode,
                sB, sT, sG, sD, sA, sub_off,
                U, H_prev, frozen, mp,
                vals_out, rvals_out,
                H_out, d_out, psi_out, sig_out,
                want_tangent):
    """AT2 phase-field assembly. U = [ux,uy]*n + d*n ; H_prev: (nsub,).

    mp: 0 lam, 1 mu, 2 gc, 3 l
    """
    ne = len(elem_ptr) - 1
    lam = mp[0]
    mu = mp[1]
    gc = mp[2]
    lf = mp[3]

    for e in prange(ne):
        m = elem_n[e]
        vids = elem_flat[elem_ptr[e]:elem_ptr[e] + m]
        ue = np.zeros(2 * MAXV)
        de = np.zeros(MAXV)
        for j in range(m):
            ue[2 * j] = U[2 * vids[j]]
            ue[2 * j + 1] = U[2 * vids[j] + 1]
            de[j] = U[2 * nnode + vids[j]]
        Kloc = np.zeros((LMAX, LMAX))
        rloc = np.zeros(LMAX)
        ns = sub_off[e]
        nf = sub_off[e + 1]
        for si in range(ns, nf):
            A_k = sA[si]
            Bk = sB[si]
            Tk = sT[si]
            Gk = sG[si]
            Dk = sD[si]
            exx = 0.0
            eyy = 0.0
            gxy = 0.0
            for c2 in range(2 * MAXV):
                exx += Bk[0, c2] * ue[c2]
                eyy += Bk[1, c2] * ue[c2]
                gxy += Bk[2, c2] * ue[c2]
            dk = 0.0
            for j in range(m):
                dk += Dk[j] * de[j]
            sp, sm, Cp, Cm, psip = spectral(exx, eyy, gxy, lam, mu)
            Hp = H_prev[si]
            if frozen:
                Hk = Hp
                loading = False
            else:
                Hk = psip if psip > Hp else Hp
                loading = psip > Hp - 1e-14
            H_out[si] = Hk
            d_out[si] = dk
            psi_out[si] = psip
            g = (1.0 - dk) ** 2
            if g < 1e-12:         # numerical floor (fully broken regions)
                g = 1e-12
            gprime = -2.0 * (1.0 - dk)
            sig = np.array([g * sp[0] + sm[0], g * sp[1] + sm[1],
                            g * sp[2] + sm[2]])
            sig_out[si, 0] = sig[0]
            sig_out[si, 1] = sig[1]
            sig_out[si, 2] = sig[2]
            for a in range(2 * m):
                rloc[a] += A_k * (Bk[0, a] * sig[0] + Bk[1, a] * sig[1]
                                  + Bk[2, a] * sig[2])
            g0 = 0.0
            g1 = 0.0
            for j in range(m):
                g0 += Gk[0, j] * de[j]
                g1 += Gk[1, j] * de[j]
            coef = -2.0 * (1.0 - dk) * Hk + (gc / lf) * dk
            for i2 in range(m):
                acc = (A_k / 3.0) * coef * Tk[i2]
                acc += gc * lf * A_k * (Gk[0, i2] * g0 + Gk[1, i2] * g1)
                rloc[2 * m + i2] += acc
            if want_tangent:
                AC = np.empty((3, 3))
                for r in range(3):
                    for c2 in range(3):
                        AC[r, c2] = g * Cp[r, c2] + Cm[r, c2]
                for a in range(2 * m):
                    for b in range(2 * m):
                        acc = 0.0
                        for r in range(3):
                            Ba = Bk[r, a]
                            if Ba != 0.0:
                                for c2 in range(3):
                                    acc += Ba * AC[r, c2] * Bk[c2, b]
                        Kloc[a, b] += A_k * acc
                if gprime != 0.0:
                    for a in range(2 * m):
                        va = (Bk[0, a] * sp[0] + Bk[1, a] * sp[1]
                              + Bk[2, a] * sp[2])
                        for j in range(m):
                            Kloc[a, 2 * m + j] += A_k * gprime * va * Dk[j]
                if loading and dk < 1.0:
                    for i2 in range(m):
                        for b in range(2 * m):
                            vb = (sp[0] * Bk[0, b] + sp[1] * Bk[1, b]
                                  + sp[2] * Bk[2, b])
                            Kloc[2 * m + i2, b] += (
                                -(A_k / 3.0) * 2.0 * (1.0 - dk) * Tk[i2] * vb)
                f1 = (A_k / 3.0) * (2.0 * Hk + gc / lf)
                for i2 in range(m):
                    for j2 in range(m):
                        Kloc[2 * m + i2, 2 * m + j2] += (
                            f1 * Tk[i2] * Dk[j2]
                            + gc * lf * A_k * (Gk[0, i2] * Gk[0, j2]
                                               + Gk[1, i2] * Gk[1, j2]))
        for a in range(LMAX):
            rvals_out[e, a] = rloc[a]
            if want_tangent:
                for b in range(LMAX):
                    vals_out[e, a, b] = Kloc[a, b]


def elastic_C(lam, mu):
    C = np.zeros((3, 3))
    C[0, 0] = lam + 2 * mu
    C[0, 1] = lam
    C[1, 0] = lam
    C[1, 1] = lam + 2 * mu
    C[2, 2] = mu
    return C
