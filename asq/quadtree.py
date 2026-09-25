"""
SDF-based quadtree mesh with 2:1 balancing, polygonal leaf cells with hanging
nodes as extra vertices, and zero-thickness slit (notch) support.

Cells are stored in a "linear quadtree" dict keyed by (level, ix, iy) where
(ix, iy) are integer coordinates on the grid of size 2^level * (nx, ny).
"""
import numpy as np


class Slit:
    """Axis-aligned zero-thickness slit (notch) from p0 to p1."""

    def __init__(self, p0, p1):
        p0 = np.asarray(p0, float)
        p1 = np.asarray(p1, float)
        if abs(p0[0] - p1[0]) < 1e-12:  # vertical
            self.dir = 'v'
            self.xc = p0[0]
            self.y0, self.y1 = sorted((p0[1], p1[1]))
        else:
            self.dir = 'h'
            self.yc = p0[1]
            self.x0, self.x1 = sorted((p0[0], p1[0]))
        self.p0, self.p1 = p0, p1
        # tip = the endpoint that is NOT on the domain boundary (set later)
        self.tip = None
        self.mouth = None

    def on_line(self, x, y, tol=1e-9):
        if self.dir == 'h':
            return abs(y - self.yc) < tol
        return abs(x - self.xc) < tol

    def inside(self, x, y, tol=1e-9):
        """Strictly inside the slit segment (not tip, not mouth)."""
        if self.dir == 'h':
            return (abs(y - self.yc) < tol
                    and self.x0 + tol < x < self.x1 - tol)
        return (abs(x - self.xc) < tol
                and self.y0 + tol < y < self.y1 - tol)

    def contains_point(self, x, y, tol=1e-9):
        if self.dir == 'h':
            return (abs(y - self.yc) < tol
                    and self.x0 - tol <= x <= self.x1 + tol)
        return (abs(x - self.xc) < tol
                and self.y0 - tol <= y <= self.y1 + tol)

    def side_of(self, x, y):
        """+1 or -1 depending on which side of the slit line the point is."""
        if self.dir == 'h':
            return 1 if y > self.yc else -1
        return 1 if x > self.xc else -1

    def bbox(self):
        if self.dir == 'h':
            return (self.x0, self.yc, self.x1, self.yc)
        return (self.xc, self.y0, self.xc, self.y1)


class Quadtree:
    def __init__(self, x0, y0, x1, y1, nx, ny, slits=(), max_depth=12,
                 mask=None):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.nx, self.ny = nx, ny
        self.max_depth = max_depth
        self.mask = mask      # callable(x0,y0,x1,y1)->bool: keep cell
        self.cells = {}  # (level, ix, iy) -> None (leaf) or dict of children
        for i in range(nx):
            for j in range(ny):
                key = (0, i, j)
                if mask is not None and not mask(*self.cell_bbox(key)):
                    continue
                self.cells[key] = None
        self.slits = list(slits)
        # resolve tip / mouth for each slit (tip = interior endpoint)
        for s in self.slits:
            for p in (s.p0, s.p1):
                on_bdry = (abs(p[0] - x0) < 1e-9 or abs(p[0] - x1) < 1e-9
                           or abs(p[1] - y0) < 1e-9 or abs(p[1] - y1) < 1e-9)
                if on_bdry:
                    s.mouth = p
                else:
                    s.tip = p
            assert s.tip is not None, "slit must have one interior endpoint (tip)"

    # ---------------- geometry helpers ----------------
    def cell_bbox(self, key):
        lvl, ix, iy = key
        wx = (self.x1 - self.x0) / (self.nx * (1 << lvl))
        wy = (self.y1 - self.y0) / (self.ny * (1 << lvl))
        return (self.x0 + ix * wx, self.y0 + iy * wy,
                self.x0 + (ix + 1) * wx, self.y0 + (iy + 1) * wy)

    def cell_center(self, key):
        a, b, c, d = self.cell_bbox(key)
        return ((a + c) / 2, (b + d) / 2)

    def cell_size(self, key):
        a, b, c, d = self.cell_bbox(key)
        return max(c - a, d - b)

    def refine_cell(self, key):
        if self.cells[key] is not None:
            return
        lvl, ix, iy = key
        if lvl >= self.max_depth:
            raise RuntimeError(f"max depth exceeded refining {key}")
        self.cells[key] = {}
        for a in (0, 1):
            for b in (0, 1):
                ck = (lvl + 1, 2 * ix + a, 2 * iy + b)
                if self.mask is not None and not self.mask(*self.cell_bbox(ck)):
                    continue
                self.cells[ck] = None

    def leaves(self):
        return [k for k, v in self.cells.items() if v is None]

    def children(self, key):
        lvl, ix, iy = key
        return [(lvl + 1, 2 * ix + a, 2 * iy + b) for a in (0, 1) for b in (0, 1)]

    def _adj_leaves(self, coord, s0, s1, orient, root):
        """Leaves among descendants of `root` touching a boundary line."""
        out = []
        stack = [root]
        while stack:
            k = stack.pop()
            if k not in self.cells:
                continue
            if self.cells[k] is None:
                out.append(k)
                continue
            for ch in self.children(k):
                a, b, c, d = self.cell_bbox(ch)
                if orient == 'v':
                    touch = (abs(a - coord) < 1e-12
                             and min(d, s1) - max(b, s0) > 1e-12)
                else:
                    touch = (abs(b - coord) < 1e-12
                             and min(c, s1) - max(a, s0) > 1e-12)
                if touch:
                    stack.append(ch)
        return out

    def finest_neighbor_level(self, key):
        """Finest leaf level among the 4 face neighbors of leaf `key`."""
        lvl, ix, iy = key
        best = lvl
        a, b, c, d = self.cell_bbox(key)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            j, k = ix + dx, iy + dy
            if j < 0 or k < 0 or j >= self.nx << lvl or k >= self.ny << lvl:
                continue  # domain boundary
            nk = (lvl, j, k)
            if nk not in self.cells:
                # neighbour is a coarser ancestor leaf
                for probe in range(lvl - 1, -1, -1):
                    scale = 1 << (lvl - probe)
                    pk = (probe, j // scale, k // scale)
                    if pk in self.cells:
                        best = max(best, probe)
                        break
                continue
            if self.cells[nk] is None:
                best = max(best, lvl)
                continue
            # refined neighbour: descend along the shared edge
            if dx == 1:
                leaves = self._adj_leaves(c, b, d, 'v', nk)
            elif dx == -1:
                leaves = self._adj_leaves(a, b, d, 'v', nk)
            elif dy == 1:
                leaves = self._adj_leaves(d, a, c, 'h', nk)
            else:
                leaves = self._adj_leaves(b, a, c, 'h', nk)
            for lf in leaves:
                best = max(best, lf[0])
        return best

    def balance(self):
        """Enforce 2:1 balance between face-neighbours (iterative)."""
        changed = True
        while changed:
            changed = False
            for key in list(self.cells.keys()):
                if self.cells[key] is not None:
                    continue
                if self.finest_neighbor_level(key) > key[0] + 1:
                    self.refine_cell(key)
                    changed = True

    # ---------------- refinement drivers ----------------
    def _size_fun_samples(self, key, hfun):
        """min of h over 9 sample points (corners, edge midpoints, centre)."""
        a, b, c, d = self.cell_bbox(key)
        mx = (a + c) / 2
        my = (b + d) / 2
        xs = (a, c, c, a, mx, mx, a, mx, c)
        ys = (b, b, d, d, b, d, my, my, my)
        try:
            return float(np.min(hfun(xs, ys)))
        except Exception:
            return min(hfun(x, y) for x, y in zip(xs, ys))

    def _violating(self, hfun):
        """Leaf keys whose size exceeds the size field at any of 9 samples."""
        keys = self.leaves()
        nk = len(keys)
        if nk == 0:
            return []
        X = np.empty((nk, 9))
        Y = np.empty((nk, 9))
        S = np.empty(nk)
        for i, key in enumerate(keys):
            a, b, c, d = self.cell_bbox(key)
            mx = (a + c) / 2
            my = (b + d) / 2
            X[i] = (a, c, c, a, mx, mx, a, mx, c)
            Y[i] = (b, b, d, d, b, d, my, my, my)
            S[i] = max(c - a, d - b)
        try:
            H = np.asarray(hfun(X.ravel(), Y.ravel()), float)
            H = H.reshape(nk, 9).min(axis=1)
        except Exception:
            H = np.array([min(hfun(x, y) for x, y in zip(X[i], Y[i]))
                          for i in range(nk)])
        return [keys[i] for i in range(nk) if S[i] > H[i] * (1 + 1e-9)]

    def refine_by_size_function(self, hfun):
        """Refine leaves whose size exceeds the target size field h(x,y)."""
        refined = 0
        while True:
            keys = self._violating(hfun)
            if not keys:
                return refined
            stack = []
            for key in keys:
                self.refine_cell(key)
                refined += 1
                stack.extend(self.children(key))
            while stack:
                key = stack.pop()
                if self.cells.get(key, 'missing') is not None:
                    continue
                if self.cell_size(key) > self._size_fun_samples(key, hfun) * (1 + 1e-9):
                    self.refine_cell(key)
                    refined += 1
                    stack.extend(self.children(key))
            self.balance()

    def align_slits(self):
        """Refine cells so slit segments lie exactly on cell edges.

        Works when slit end coordinates are dyadic w.r.t. the root grid
        (guaranteed by benchmark setup).
        """
        for _ in range(self.max_depth):
            bad = []
            for key in self.leaves():
                a, b, c, d = self.cell_bbox(key)
                for s in self.slits:
                    # cell strictly crossed by the slit interior
                    if s.dir == 'h':
                        if (b < s.yc < d and a < s.x1 and c > s.x0
                                and not (abs(b - s.yc) < 1e-9 or abs(d - s.yc) < 1e-9)):
                            bad.append(key)
                            break
                    else:
                        if (a < s.xc < c and b < s.y1 and d > s.y0
                                and not (abs(a - s.xc) < 1e-9 or abs(c - s.xc) < 1e-9)):
                            bad.append(key)
                            break
            if not bad:
                return
            for key in bad:
                self.refine_cell(key)
            self.balance()
        raise RuntimeError("could not align slits on quadtree edges")


# ----------------------------------------------------------------------
#  mesh build: leaves -> polygonal elements with hanging-node vertices
# ----------------------------------------------------------------------
class Mesh:
    """Polygonal cell mesh built from a balanced quadtree.

    Elements are the quadtree leaves; vertices are cell corners plus
    mid-side hanging nodes (when the neighbour across an edge is finer).
    Vertices lying strictly inside a slit (or at its mouth) are duplicated
    per slit side; the tip node is shared.
    """

    def __init__(self, tree: Quadtree):
        self.tree = tree
        self._quant = None
        self._build()

    # -- vertex bookkeeping ------------------------------------------------
    def _vkey(self, x, y, side):
        q = self._quant
        kx = int(round(x / q))
        ky = int(round(y / q))
        return (kx, ky, side)

    def _add_vertex(self, x, y, side=0):
        key = self._vkey(x, y, side)
        vid = self.vmap.get(key)
        if vid is None:
            vid = len(self.xy)
            self.vmap[key] = vid
            self.xy.append((x, y))
        return vid

    def _vertex_on_slit(self, x, y):
        for si, s in enumerate(self.tree.slits):
            if s.contains_point(x, y):
                # tip is shared -> side 0; interior/mouth -> duplicated
                if s.tip is not None and (abs(x - s.tip[0]) < 1e-9
                                          and abs(y - s.tip[1]) < 1e-9):
                    return si, 0
                return si, 1  # needs duplication
        return -1, 0

    def _build(self):
        t = self.tree
        # quantisation = finest representable size
        self._quant = min((t.x1 - t.x0) / (t.nx * (1 << t.max_depth)),
                          (t.y1 - t.y0) / (t.ny * (1 << t.max_depth))) * 1.0
        self.vmap = {}
        self.xy = []
        self.elems = []      # list of vertex lists (CCW)
        self.elem_keys = []  # quadtree key per element (for inheritance)

        # corner coordinates of a leaf
        def corners(key):
            a, b, c, d = t.cell_bbox(key)
            return [(a, b), (c, b), (c, d), (a, d)]

        def neighbor_finer(key, edge):
            """Is the leaf across `edge` (0=right,1=left,2=top,3=bottom) finer?"""
            lvl, ix, iy = key
            if edge == 0:
                j, k = ix + 1, iy
            elif edge == 1:
                j, k = ix - 1, iy
            elif edge == 2:
                j, k = ix, iy + 1
            else:
                j, k = ix, iy - 1
            if j < 0 or k < 0 or j >= t.nx << lvl or k >= t.ny << lvl:
                return False
            for probe in range(lvl, -1, -1):
                scale = 1 << (lvl - probe)
                pk = (probe, j // scale, k // scale)
                if pk in t.cells:
                    return t.cells[pk] is not None  # refined => finer leaves
            return False

        def get_vid(x, y, cell_center):
            si, dup = self._vertex_on_slit(x, y)
            if dup:
                s = t.slits[si]
                side = s.side_of(*cell_center)
                # mouth node: two copies (one per side) -> use side tag
                return self._add_vertex(x, y, side)
            return self._add_vertex(x, y, 0)

        for key in t.leaves():
            cx, cy = t.cell_center(key)
            cs = corners(key)  # CCW: bl, br, tr, tl
            # polygon vertices: corners with midpoints on edges adjacent to
            # finer neighbours
            poly = []
            # edges: (bl->br) bottom, (br->tr) right, (tr->tl) top, (tl->bl) left
            edge_defs = [(0, 1, 3), (1, 2, 0), (2, 3, 2), (3, 0, 1)]
            for ia, ib, ecode in edge_defs:
                pa, pb = cs[ia], cs[ib]
                poly.append(get_vid(*pa, (cx, cy)))
                if neighbor_finer(key, ecode):
                    pm = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
                    poly.append(get_vid(*pm, (cx, cy)))
            self.elems.append(poly)
            self.elem_keys.append(key)

        self.xy = np.array(self.xy, float)
        self.nelem = len(self.elems)
        self.nnode = len(self.xy)

    # -- point location (for state transfer) -------------------------------
    def locate(self, x, y):
        """Find the leaf cell key containing (x, y); None if outside."""
        t = self.tree
        if not (t.x0 - 1e-9 <= x <= t.x1 + 1e-9
                and t.y0 - 1e-9 <= y <= t.y1 + 1e-9):
            return None
        # start at root level
        lvl = 0
        ix = min(int((x - t.x0) / (t.x1 - t.x0) * t.nx), t.nx - 1)
        iy = min(int((y - t.y0) / (t.y1 - t.y0) * t.ny), t.ny - 1)
        ix, iy = max(ix, 0), max(iy, 0)
        key = (0, ix, iy)
        while t.cells.get(key, 'missing') is not None:
            a, b, c, d = t.cell_bbox(key)
            lvl1 = key[0] + 1
            hx = (c - a) / 2
            hy = (d - b) / 2
            j = 0 if x < a + hx else 1
            k = 0 if y < b + hy else 1
            key = (lvl1, 2 * key[1] + j, 2 * key[2] + k)
        if key in t.cells:
            return key
        # x or y exactly on a boundary between roots: nudge
        return None

    def elem_index_of_key(self):
        return {k: i for i, k in enumerate(self.elem_keys)}


# ----------------------------------------------------------------------
#  SDF size fields
# ----------------------------------------------------------------------
def feature_size_function(points, h_min, h_max, grade=1.0, r_buf=0.0):
    """h(x) = clip(h_min + grade * max(dist(x, points) - r_buf, 0), h_min, h_max).

    Accepts scalars or arrays for x, y."""
    from scipy.spatial import cKDTree
    pts = np.asarray(points, float).reshape(-1, 2)
    if len(pts) == 0:
        return lambda x, y: np.full(np.shape(x), h_max) if hasattr(x, '__len__') else h_max
    tree = cKDTree(pts)

    def hfun(x, y):
        d, _ = tree.query(np.column_stack([np.atleast_1d(np.asarray(x, float)),
                                           np.atleast_1d(np.asarray(y, float))]))
        h = np.clip(h_min + grade * np.maximum(d - r_buf, 0.0), h_min, h_max)
        if np.ndim(x) == 0 and np.ndim(y) == 0:
            return float(h[0])
        return h

    return hfun


def multi_feature_size_function(features, h_min, h_max):
    """features: list of (points, grade, r_buf); take the min over features."""
    from scipy.spatial import cKDTree
    fdat = []
    for pts, grade, r_buf in features:
        P = np.asarray(pts, float).reshape(-1, 2)
        if len(P):
            fdat.append((cKDTree(P), grade, r_buf))

    def hfun(x, y):
        xa = np.atleast_1d(np.asarray(x, float))
        ya = np.atleast_1d(np.asarray(y, float))
        Q = np.column_stack([xa, ya])
        h = np.full(len(xa), h_max)
        for tree, grade, r_buf in fdat:
            d, _ = tree.query(Q)
            h = np.minimum(h, np.clip(h_min + grade * np.maximum(d - r_buf, 0.0),
                                      h_min, h_max))
        if np.ndim(x) == 0 and np.ndim(y) == 0:
            return float(h[0])
        return h

    return hfun


def combine_hfuns(*hfuns):
    def hfun(x, y):
        return min(h(x, y) for h in hfuns)
    return hfun


def box_size_function(bx0, by0, bx1, by1, h_min, h_max, grade=1.0):
    """h_min inside the box, graded outward by distance to the box."""

    def hfun(x, y):
        xa = np.atleast_1d(np.asarray(x, float))
        ya = np.atleast_1d(np.asarray(y, float))
        dx = np.maximum(np.maximum(bx0 - xa, xa - bx1), 0.0)
        dy = np.maximum(np.maximum(by0 - ya, ya - by1), 0.0)
        d = np.hypot(dx, dy)
        h = np.clip(h_min + grade * d, h_min, h_max)
        if np.ndim(x) == 0 and np.ndim(y) == 0:
            return float(h[0])
        return h

    return hfun


def slab_size_function(x0, y0, x1, y1, h_min, h_max, grade, width):
    """Band of `width` around a segment refined to h_min, graded to h_max."""
    import math

    def hfun(x, y):
        # distance to segment
        ax, ay = x0, y0
        bx, by = x1, y1
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        return min(max(h_min + grade * max(d - width, 0.0), h_min), h_max)

    return hfun
