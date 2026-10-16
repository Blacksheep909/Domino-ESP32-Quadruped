"""Close-up re-render of saved von Mises results (no re-solve). usage: python render.py <tag> [case ...]"""
import sys, os, json
import numpy as np
import gmsh
from skfem import MeshTet
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
GEOM = os.path.join(HERE, "geometry")
OUT = os.path.join(HERE, "results", sys.argv[1] if len(sys.argv) > 1 else "v9")
os.makedirs(OUT, exist_ok=True)
TAG = sys.argv[1]
HMAX = 1.6
CASES = {c["key"]: c["name"] for c in json.load(open(os.path.join(HERE, "loads", "loadcases.json")))}
keys = sys.argv[2:] or list(CASES)

# identical meshing to fea.py
gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
gmsh.model.occ.importShapes(os.path.join(GEOM, TAG + ".step")); gmsh.model.occ.synchronize()
info = sorted(((gmsh.model.occ.getMass(d, t), t) for d, t in gmsh.model.getEntities(3)), reverse=True)
gmsh.model.occ.remove([(3, t) for _, t in info[1:]], recursive=True); gmsh.model.occ.synchronize()
gmsh.option.setNumber("Mesh.MeshSizeMax", HMAX); gmsh.option.setNumber("Mesh.MeshSizeMin", 0.45)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 14); gmsh.option.setNumber("Mesh.Algorithm3D", 10)
gmsh.option.setNumber("Mesh.Optimize", 1)
gmsh.model.mesh.generate(3)
nt, xyz, _ = gmsh.model.mesh.getNodes(); xyz = xyz.reshape(-1, 3)
idx = {int(t): i for i, t in enumerate(nt)}
_, en = gmsh.model.mesh.getElementsByType(4)
tets = np.array([idx[int(n)] for n in en]).reshape(-1, 4)
gmsh.finalize()
used = np.unique(tets); remap = -np.ones(len(xyz), dtype=int); remap[used] = np.arange(len(used))
mesh = MeshTet(xyz[used].T, remap[tets].T)
bf = mesh.boundary_facets(); f2t = mesh.f2t[0, bf]; tris = mesh.facets[:, bf].T
P = mesh.p.T

VIEWS = [(28, -58, "Iso (front-right)"), (28, 122, "Iso (back-left)"), (90, -90, "Top (+Z)"), (0, -90, "Front (-Y): shaft clamps"),
         (-28, -58, "Iso from below"), (0, 0, "End view (+X)")]

from scipy.spatial import cKDTree
saved_c = np.load(os.path.join(OUT, "%s_centroids.npy" % TAG)).T          # (ne_saved, 3)
dist, nearest = cKDTree(saved_c).query(mesh.p[:, mesh.t].mean(axis=1).T)
print("mapped %d elements to saved results; median offset %.4f mm, max %.3f mm" % (len(nearest), np.median(dist), dist.max()))

for key in keys:
    vm = np.load(os.path.join(OUT, "%s_vm_%s.npy" % (TAG, key)))[nearest]
    vmax = float(np.percentile(vm, 99.9))
    vals = vm[f2t]
    cmap = plt.get_cmap("turbo"); norm = matplotlib.colors.Normalize(0, vmax)
    fig = plt.figure(figsize=(18, 12), dpi=130)
    mn, mx = P.min(0), P.max(0); c = (mn + mx) / 2; ext = mx - mn
    for k, (el, az, lab) in enumerate(VIEWS):
        ax = fig.add_subplot(2, 3, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(P[tris], facecolors=cmap(norm(vals)), edgecolors="none", linewidths=0))
        ax.set_xlim(mn[0], mx[0]); ax.set_ylim(mn[1], mx[1]); ax.set_zlim(mn[2], mx[2])
        ax.set_box_aspect(ext, zoom=1.08)
        ax.view_init(el, az); ax.set_axis_off(); ax.set_title(lab, fontsize=12, pad=-4)
    fig.subplots_adjust(left=0.0, right=0.9, bottom=0.0, top=0.93, wspace=0.02, hspace=0.06)
    cax = fig.add_axes([0.92, 0.2, 0.015, 0.6])
    cb = fig.colorbar(matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax); cb.set_label("von Mises stress (MPa)", fontsize=12)
    fig.suptitle("%s - %s   |   peak %.2f MPa   |   SF %.0f in-layer (50 MPa), %.0f inter-layer (25 MPa)" % (
        TAG, CASES[key], vm.max(), 50 / vm.max(), 25 / vm.max()), fontsize=14)
    out = os.path.join(OUT, "%s_%s_close.png" % (TAG, key))
    fig.savefig(out); plt.close(fig)
    print("saved", out)

