"""
Cell-based strain smoothing (beta-CS) on polygonal quadtree cells.

Each polygonal cell (4..8 vertices: corners + hanging mid-side nodes) is
split into `m` triangular sub-smoothing-domains (v_k, v_{k+1}, centroid).
The centroid carries no degree of freedom; its value is the minimum-norm
linear-consistent combination of vertex values (weights w).

Per sub-domain k we precompute (all expressed on the cell's vertex dofs):
  sB[k]  (3 x 2m)  effective strain-displacement matrix of the constant
                   (smoothed) sub-triangle strain, beta-stabilised
  sT[k]  (1 x m)   P1 interpolation row: scalar at (v_k, v_{k+1}, centroid)
  sG[k]  (2 x m)   gradient of the P1 scalar field on the sub-triangle
  sM[k]  (m x m)   consistent mass  T^T M_tri T  of the sub-triangle
  sD[k]  (1 x m)   scalar value at the sub-triangle centroid
"""
import numpy as np
from numba import njit

MAXV = 8


@njit(cache=True)
def _solve3(A, b):
    """Solve 3x3 system (A symmetric pos def)."""
    M = A.copy()
    x = b.copy()
    # Cholesky-like elimination without sqrt (LDL)
    for i in range(3):
        # pivot
        if abs(M[i, i]) < 1e-300:
            M[i, i] = 1e-300
        for j in range(i + 1, 3):
            f = M[j, i] / M[i, i]
            for k in range(i + 1, 3):
                M[j, k] -= f * M[i, k]
            x[j] -= f * x[i]
    x[2] = x[2] / M[2, 2]
    x[1] = (x[1] - M[1, 2] * x[2]) / M[1, 1]
    x[0] = (x[0] - M[0, 1] * x[1] - M[0, 2] * x[2]) / M[0, 0]
    return x


@njit(cache=True)
def precompute(elem_flat, elem_ptr, elem_n, xy, beta):
    ne = len(elem_ptr) - 1
    maxv = MAXV
    nsub_tot = 0
    for e in range(ne):
        nsub_tot += elem_n[e]

    sB = np.zeros((nsub_tot, 3, 2 * maxv))
    sT = np.zeros((nsub_tot, maxv))
    sG = np.zeros((nsub_tot, 2, maxv))
    sM = np.zeros((nsub_tot, maxv, maxv))
    sD = np.zeros((nsub_tot, maxv))
    sA = np.zeros(nsub_tot)
    sC = np.zeros((nsub_tot, 2))
    eW = np.zeros((ne, maxv))
    eA = np.zeros(ne)
    sub_el = np.zeros(nsub_tot, np.int32)
    sub_k = np.zeros(nsub_tot, np.int32)

    A3 = np.zeros((3, MAXV))
    AAt = np.zeros((3, 3))
    b3 = np.zeros(3)

    s = 0
    for e in range(ne):
        m = elem_n[e]
        vids = elem_flat[elem_ptr[e]:elem_ptr[e] + m]
        cx = 0.0
        cy = 0.0
        for j in range(m):
            cx += xy[vids[j], 0]
            cy += xy[vids[j], 1]
        cx /= m
        cy /= m
        # --- min-norm linear-consistent centroid weights ---
        # w = A^T (A A^T)^{-1} b,  A = [1; x; y] (3 x m), b = [1; cx; cy]
        for j in range(m):
            A3[0, j] = 1.0
            A3[1, j] = xy[vids[j], 0]
            A3[2, j] = xy[vids[j], 1]
        b3[0] = 1.0
        b3[1] = cx
        b3[2] = cy
        for r in range(3):
            for c in range(3):
                acc = 0.0
                for k in range(m):
                    acc += A3[r, k] * A3[c, k]
                AAt[r, c] = acc
        z = _solve3(AAt, b3)
        w = np.empty(maxv)
        for k in range(m):
            acc = 0.0
            for r in range(3):
                acc += A3[r, k] * z[r]
            w[k] = acc
        for j in range(m):
            eW[e, j] = w[j]

        area_e = 0.0
        for k in range(m):
            k1 = (k + 1) % m
            v0 = vids[k]
            v1 = vids[k1]
            x0, y0 = xy[v0, 0], xy[v0, 1]
            x1, y1 = xy[v1, 0], xy[v1, 1]
            # signed area*2 and B of triangle (v_k, v_k1, centroid)
            det = (x1 - x0) * (cy - y0) - (cx - x0) * (y1 - y0)
            if abs(det) < 1e-30:
                det = 1e-30
            area = 0.5 * det
            d = 1.0 / det
            # triangle node gradients
            # N1 (v_k):  ((y1-cy), (cx-x1)) / det
            # N2 (v_k1): ((cy-y0), (x0-cx)) / det
            # N3 (c):    ((y0-y1), (x1-x0)) / det
            Bk = np.zeros((3, 2 * maxv))
            # B rows: [0]=dN/dx (exx), [1]=dN/dy (eyy),
            #         [2]: B[2,2j]=dN/dy, B[2,2j+1]=dN/dx  (gamma_xy)
            # node v_k (N1): dN/dx=(y1-cy)/det, dN/dy=(cx-x1)/det
            Bk[0, 2 * k] = (y1 - cy) * d
            Bk[1, 2 * k + 1] = (cx - x1) * d
            Bk[2, 2 * k] = (cx - x1) * d
            Bk[2, 2 * k + 1] = (y1 - cy) * d
            # node v_k1 (N2): dN/dx=(cy-y0)/det, dN/dy=(x0-cx)/det
            Bk[0, 2 * k1] = (cy - y0) * d
            Bk[1, 2 * k1 + 1] = (x0 - cx) * d
            Bk[2, 2 * k1] = (x0 - cx) * d
            Bk[2, 2 * k1 + 1] = (cy - y0) * d
            # centroid (N3): dN/dx=(y0-y1)/det, dN/dy=(x1-x0)/det
            for j in range(m):
                Bk[0, 2 * j] += w[j] * (y0 - y1) * d
                Bk[1, 2 * j + 1] += w[j] * (x1 - x0) * d
                Bk[2, 2 * j] += w[j] * (x1 - x0) * d
                Bk[2, 2 * j + 1] += w[j] * (y0 - y1) * d
            sB[s] = Bk
            # T row: value at (v_k, v_k1, centroid=w)
            Tk = np.zeros(maxv)
            Tk[k] = 1.0
            Tk[k1] += 1.0
            for j in range(m):
                Tk[j] += w[j]
            sT[s] = Tk
            # gradient rows
            Gk = np.zeros((2, maxv))
            Gk[0, k] = (y1 - cy) * d
            Gk[1, k] = (cx - x1) * d
            Gk[0, k1] += (cy - y0) * d
            Gk[1, k1] += (x0 - cx) * d
            for j in range(m):
                Gk[0, j] += w[j] * (y0 - y1) * d
                Gk[1, j] += w[j] * (x1 - x0) * d
            sG[s] = Gk
            # consistent mass: M_eff[i,j] = A/12 (Tk[i] Tk[j] + Ti.Tj)
            # Ti (tri-node space) = (e_k[i], e_k1[i], w[i])
            for i2 in range(m):
                for j2 in range(m):
                    dot = 0.0
                    if i2 == k and j2 == k:
                        dot += 1.0
                    if i2 == k1 and j2 == k1:
                        dot += 1.0
                    dot += w[i2] * w[j2]
                    sM[s, i2, j2] = area / 12.0 * (Tk[i2] * Tk[j2] + dot)
            # D row: centroid value = (v_k + v_k1 + centroid)/3
            Dk = np.zeros(maxv)
            Dk[k] = 1.0 / 3.0
            Dk[k1] += 1.0 / 3.0
            for j in range(m):
                Dk[j] += w[j] / 3.0
            sD[s] = Dk
            sA[s] = area
            sC[s, 0] = (x0 + x1 + cx) / 3.0
            sC[s, 1] = (y0 + y1 + cy) / 3.0
            sub_el[s] = e
            sub_k[s] = k
            area_e += area
            s += 1
        eA[e] = area_e

    # ---- beta stabilisation:  B_k^b = Bbar + sqrt(beta) (B_k - Bbar) ----
    sb = 0
    f = np.sqrt(beta)
    for e in range(ne):
        ns = elem_n[e]
        A_e = eA[e]
        if A_e <= 0:
            A_e = 1e-30
        Bbar = np.zeros((3, 2 * maxv))
        for k in range(ns):
            for r in range(3):
                for c in range(2 * maxv):
                    Bbar[r, c] += sA[sb + k] * sB[sb + k, r, c]
        for r in range(3):
            for c in range(2 * maxv):
                Bbar[r, c] /= A_e
        for k in range(ns):
            for r in range(3):
                for c in range(2 * maxv):
                    sB[sb + k, r, c] = (Bbar[r, c]
                                        + f * (sB[sb + k, r, c] - Bbar[r, c]))
        sb += ns

    return (sB, sT, sG, sM, sD, sA, sC, eW, eA, sub_el, sub_k)


def build_sfem(mesh, beta=1.0):
    ne = mesh.nelem
    counts = np.array([len(p) for p in mesh.elems], np.int64)
    ptr = np.zeros(ne + 1, np.int64)
    ptr[1:] = np.cumsum(counts)
    flat = np.zeros(int(ptr[-1]), np.int64)
    for e, p in enumerate(mesh.elems):
        flat[ptr[e]:ptr[e] + len(p)] = p
    (sB, sT, sG, sM, sD, sA, sC, eW, eA, sub_el, sub_k) = precompute(
        flat, ptr, counts, mesh.xy, beta)
    nsub = len(sA)
    return dict(elem_flat=flat, elem_ptr=ptr, elem_n=counts, sB=sB, sT=sT,
                sG=sG, sM=sM, sD=sD, sA=sA, sC=sC, eW=eW, eA=eA,
                sub_el=sub_el, sub_k=sub_k, nsub=nsub, beta=beta)
