"""Linear static FEA of the generative bracket under the Domino quadruped pin-reaction load cases.
usage: python fea.py <tag> [hmax_mm]
Units: mm, N, MPa.  Model frame = Fusion design frame (cm) * 10.
"""
import sys, os, json, math, time
import numpy as np
import gmsh
import scipy.sparse.linalg as spla
import pyamg
from skfem import MeshTet, Basis, ElementTetP2, ElementVector, FacetBasis, asm, condense
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot
from skfem import LinearForm
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
GEOM = os.path.join(HERE, "geometry")
OUT = os.path.join(HERE, "results", sys.argv[1] if len(sys.argv) > 1 else "v9")
os.makedirs(OUT, exist_ok=True)
TAG = sys.argv[1] if len(sys.argv) > 1 else "v9"
HMAX = float(sys.argv[2]) if len(sys.argv) > 2 else 1.6
E, NU = 2000.0, 0.38                 # PETG (FDM, approx.)
YIELD_XY, YIELD_Z = 50.0, 25.0       # in-layer / inter-layer strength estimates
PAD_A = (-17.76, -170.26)            # driven L2 pin ("right")
PAD_B = (-57.76, -170.27)            # passive push-rod pin ("left")
PAD_Z = [(22.1, 28.1), (48.1, 54.1)]
SHAFT = (-190.28, 38.1)              # L3 tube axis (y, z), along X
CLAMP_X = [(-67.76, -47.75), (-27.76, -7.75)]

with open(os.path.join(HERE, "loads", "loadcases.json")) as f:
    CASES = json.load(f)

# ---------------------------------------------------------------- mesh
t0 = time.time()
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
gmsh.model.occ.importShapes(os.path.join(GEOM, TAG + ".step"))
gmsh.model.occ.synchronize()
vols = gmsh.model.getEntities(3)
info = []
for dim, tag in vols:
    m = gmsh.model.occ.getMass(dim, tag)
    bb = gmsh.model.getBoundingBox(dim, tag)
    info.append((m, tag, bb))
info.sort(reverse=True)
keep = info[0]
print("volumes in STEP:", [(round(m), t) for m, t, _ in info], "-> keeping", keep[1], "vol %.0f mm3" % keep[0])
gmsh.model.occ.remove([(3, t) for _, t, _ in info[1:]], recursive=True)
gmsh.model.occ.synchronize()
gmsh.option.setNumber("Mesh.MeshSizeMax", HMAX)
gmsh.option.setNumber("Mesh.MeshSizeMin", 0.45)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 14)
gmsh.option.setNumber("Mesh.Algorithm3D", 10)          # HXT
gmsh.option.setNumber("Mesh.Optimize", 1)
gmsh.model.mesh.generate(3)
nt, xyz, _ = gmsh.model.mesh.getNodes()
xyz = xyz.reshape(-1, 3)
idx = {int(t): i for i, t in enumerate(nt)}
et, en = gmsh.model.mesh.getElementsByType(4)
tets = np.array([idx[int(n)] for n in en]).reshape(-1, 4)
gmsh.finalize()
used = np.unique(tets)
remap = -np.ones(len(xyz), dtype=int); remap[used] = np.arange(len(used))
p = xyz[used].T
t = remap[tets].T
mesh = MeshTet(p, t)
print("mesh: %d nodes, %d tets (%.0fs)" % (p.shape[1], t.shape[1], time.time() - t0))

# ---------------------------------------------------------------- FE setup
e = ElementVector(ElementTetP2())
ib = Basis(mesh, e, intorder=2)
lam, mu = lame_parameters(E, NU)
t1 = time.time()
K = asm(linear_elasticity(lam, mu), ib)
print("assembled K: %d dofs (%.0fs)" % (K.shape[0], time.time() - t1))

mid = mesh.p[:, mesh.facets].mean(axis=1)            # facet midpoints (3, nf)
bf = mesh.boundary_facets()
mb = mid[:, bf]

def on_cyl_z(c, r, tol, zr):
    d = np.hypot(mb[0] - c[0], mb[1] - c[1])
    inz = np.zeros(mb.shape[1], bool)
    for z0, z1 in zr:
        inz |= (mb[2] > z0 - 0.05) & (mb[2] < z1 + 0.05)
    return bf[(np.abs(d - r) < tol) & inz]

def pin_facets(c):
    # bolt-hole wall in both pads: find the hole radius present in this model
    d = np.hypot(mb[0] - c[0], mb[1] - c[1])
    inz = np.zeros(mb.shape[1], bool)
    for z0, z1 in PAD_Z:
        inz |= (mb[2] > z0 - 0.05) & (mb[2] < z1 + 0.05)
    cand = d[inz & (d < 2.5)]
    r = np.median(cand) if len(cand) else 1.57
    return on_cyl_z(c, r, 0.25, PAD_Z), r

dsh = np.hypot(mb[1] - SHAFT[0], mb[2] - SHAFT[1])
inx = np.zeros(mb.shape[1], bool)
for x0, x1 in CLAMP_X:
    inx |= (mb[0] > x0 - 0.1) & (mb[0] < x1 + 0.1)
fixed_f = bf[(np.abs(dsh - 4.1) < 0.3) & inx]
fa, ra = pin_facets(PAD_A)
fb, rb = pin_facets(PAD_B)
print("fixed bore facets %d | pin A facets %d (hole r %.2f mm) | pin B facets %d (hole r %.2f mm)" % (len(fixed_f), len(fa), ra, len(fb), rb))
assert len(fixed_f) > 50 and len(fa) > 20 and len(fb) > 20

fixed_dofs = ib.get_dofs(facets=fixed_f).all()

def traction_vec(facets, F):
    fbas = FacetBasis(mesh, e, facets=facets, intorder=3)
    area = asm(LinearForm(lambda v, w: 1.0 + 0 * w.x[0]), FacetBasis(mesh, ElementTetP2(), facets=facets)).sum()
    tvec = np.array([F[0], F[1], 0.0]) / area
    @LinearForm
    def lf(v, w):
        return tvec[0] * v[0] + tvec[1] * v[1] + tvec[2] * v[2]
    return asm(lf, fbas), area

Kc_cache = {}
def solve(fvec):
    Kc, fc, x, I = condense(K, fvec, D=fixed_dofs)
    if "ml" not in Kc_cache:
        Kc_cache["ml"] = pyamg.smoothed_aggregation_solver(Kc.tocsr(), B=None, max_coarse=2000)
    M = Kc_cache["ml"].aspreconditioner()
    xs, info = spla.cg(Kc, fc, rtol=1e-8, maxiter=4000, M=M)
    if info != 0:
        print("  CG info", info)
    u = x.copy(); u[I] = xs
    return u

# von Mises per element (max over quadrature points)
def von_mises(u):
    w = ib.interpolate(u)
    G = w.grad                                  # (3,3,ne,nq)
    eps = 0.5 * (G + np.transpose(G, (1, 0, 2, 3)))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    s = 2 * mu * eps
    for i in range(3):
        s[i, i] += lam * tr
    vm = np.sqrt(0.5 * ((s[0, 0] - s[1, 1]) ** 2 + (s[1, 1] - s[2, 2]) ** 2 + (s[2, 2] - s[0, 0]) ** 2)
                 + 3 * (s[0, 1] ** 2 + s[1, 2] ** 2 + s[0, 2] ** 2))
    return vm.max(axis=1)

# surface triangles + owning element for plotting
f2t = mesh.f2t[0, bf]
tris = mesh.facets[:, bf].T

def render(vm_e, title, path, vmax):
    vals = vm_e[f2t]
    P = mesh.p.T
    fig = plt.figure(figsize=(16, 11), dpi=110)
    views = [(25, -60, "iso"), (25, 120, "iso (back)"), (90, -90, "top (+Z)"), (0, -90, "front (-Y)")]
    cmap = plt.get_cmap("turbo")
    norm = matplotlib.colors.Normalize(0, vmax)
    for k, (el, az, lab) in enumerate(views):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        pc = Poly3DCollection(P[tris], facecolors=cmap(norm(vals)), edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
        mn, mx = P.min(0), P.max(0); c = (mn + mx) / 2; h = (mx - mn).max() / 2
        ax.set_xlim(c[0] - h, c[0] + h); ax.set_ylim(c[1] - h, c[1] + h); ax.set_zlim(c[2] - h, c[2] + h)
        ax.view_init(el, az); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off(); ax.set_title(lab, fontsize=11)
    sm = matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap)
    cb = fig.colorbar(sm, ax=fig.axes, shrink=0.6, pad=0.02); cb.set_label("von Mises stress (MPa)")
    fig.suptitle(title, fontsize=13)
    fig.savefig(path); plt.close(fig)

results = {"tag": TAG, "nodes": int(p.shape[1]), "tets": int(t.shape[1]), "dofs": int(K.shape[0]), "cases": []}
centroids = mesh.p[:, mesh.t].mean(axis=1)
for case in CASES:
    t2 = time.time()
    fa_vec, aa = traction_vec(fa, case["A"])
    fb_vec, ab = traction_vec(fb, case["B"])
    u = solve(fa_vec + fb_vec)
    vm = von_mises(u)
    umag = np.sqrt(u[ib.nodal_dofs[0]] ** 2 + u[ib.nodal_dofs[1]] ** 2 + u[ib.nodal_dofs[2]] ** 2)
    order = np.argsort(vm)[::-1]
    p99 = float(np.percentile(vm, 99.9))
    top = []
    for i in order[:400]:
        c = centroids[:, i]
        if all(np.linalg.norm(c - np.array(q["at"])) > 4.0 for q in top):
            top.append({"vm": round(float(vm[i]), 2), "at": [round(float(v), 2) for v in c]})
        if len(top) == 5:
            break
    r = {"name": case["name"], "max_vm": round(float(vm.max()), 2), "p99_9_vm": round(p99, 2),
         "max_disp_mm": round(float(umag.max()), 4), "SF_inlayer": round(YIELD_XY / vm.max(), 2), "SF_interlayer": round(YIELD_Z / vm.max(), 2),
         "hotspots": top, "pin_areas_mm2": [round(aa, 2), round(ab, 2)]}
    results["cases"].append(r)
    print("%-36s max VM %6.2f MPa (99.9%% %5.2f) disp %.3f mm  SF %.1f / %.1f  (%.0fs)" % (
        case["name"], r["max_vm"], p99, r["max_disp_mm"], r["SF_inlayer"], r["SF_interlayer"], time.time() - t2))
    np.save(os.path.join(OUT, "%s_vm_%s.npy" % (TAG, case["key"])), vm)
    render(vm, "%s - %s   |  max %.1f MPa, SF %.1f (in-layer 50 MPa)" % (TAG, case["name"], vm.max(), YIELD_XY / vm.max()),
           os.path.join(OUT, "%s_%s.png" % (TAG, case["key"])), vmax=max(p99, 1e-6))
np.save(os.path.join(OUT, "%s_centroids.npy" % TAG), centroids)
json.dump(results, open(os.path.join(OUT, "%s_results.json" % TAG), "w"), indent=1)
print("done in %.0fs" % (time.time() - t0))
