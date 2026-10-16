"""The 3D view's geometry, tessellated from the solids the STEP file is made of.

:func:`~cycloidgen.viz.mesh.build_mesh` draws the gearbox from the closed-form
profile in milliseconds, which is what lets the view follow a slider.  What it
draws is the gearbox's *prisms*: no edge breaks, no lead-ins, the disc as a
polygon, a bearing as a ring.  The exported solids have all of that, and they
are what a part is made from - so once a design has settled, the view swaps its
quick mesh for this one, which is those solids, triangulated finely.

It comes back as a :class:`~cycloidgen.viz.mesh.Mesh` with the quick mesh's own
parts - the same names, groups, colours and motions - so everything downstream
of a mesh takes it unchanged: the section that caps it, the edges, the picking,
the X-ray, the explode.  Each part is carried from where the assembly puts it,
at crank zero, back into the frame the quick mesh builds that part in, by the
inverse of the pose :meth:`~cycloidgen.viz.mesh.Mesh.world_vertices` applies.
That is checked rather than trusted: ``tests/test_cadmesh.py`` holds every part
to the volume of its solid and to the place the quick mesh puts it.

Needs the CAD kernel.  A build without it raises ``ImportError`` here and the
view keeps the quick mesh, which is the whole of what it had before.
"""
from __future__ import annotations

import math
import threading
from collections import OrderedDict
from dataclasses import astuple, is_dataclass, replace

import numpy as np

from ..core.spec import GearSpec
from .mesh import Mesh, Part, mesh_fingerprint

__all__ = ["ANGULAR_TOLERANCE", "LINEAR_TOLERANCE", "SOLID_ONLY", "cached",
           "cad_key", "cad_mesh"]

#: How far a triangle may stand off the true surface, millimetres.  A
#: hundredth puts the housing's rim at about 180 facets and its silhouette
#: smooth at any zoom the view allows.
LINEAR_TOLERANCE = 0.01
#: And how far a facet may turn from the next, radians - what decides a small
#: circle, where the linear limit is met by a handful of facets.  About forty
#: to a pin or a bolt.
ANGULAR_TOLERANCE = 0.15
#: Coarser for the bought parts, whose detail is small and many: a bearing's
#: needles and balls and a bolt's thread grooves are most of the triangles in
#: the drive at the finer setting, and twenty facets round a 1.5 mm needle is
#: round at any zoom the view allows.
BOUGHT_ANGULAR_TOLERANCE = 0.3
BOUGHT_GROUPS = frozenset({"bearings", "fasteners"})

#: What the solids read off the design that the quick mesh does not, so is not
#: in its fingerprint: the edge breaks, the shaft's flat, the disc as a spline,
#: the bolts by size and length.  Some are derived from fields the fingerprint
#: has, and are here anyway - a key that is right by derivation is right until
#: the derivation changes.  ``tests/test_cadmesh.py`` reads every ``spec.``
#: in :mod:`cycloidgen.export.solid` and fails on one that is in neither.
SOLID_ONLY = (
    "barrel_bottom", "barrel_height", "base_plate_bottom", "disc_hole_phases",
    "disc_phases", "discs_are_identical", "dxf_chord_tolerance", "edge_chamfer",
    "edge_break", "rim_land", "square_housing", "housing_corner_radius",
    "tie_bolt_centres",
    "has_motor_face", "housing_wall", "input_flat", "motor_face",
    "motor_bolt_centres", "motor_side", "coupled", "plate_face_z",
    "motor_mount_hole", "motor_mount_flange", "motor_hub_wall", "window_span",
    "window_half_width",
    "output_bolt_phase", "output_face_bolt_radius", "output_hole_diameter",
    "pin_count", "ratio", "ring_pin_length", "stack_height",
    "stl_linear_tolerance", "tie_bolt_bottom", "tie_bolt_length",
    "tie_bolt_size", "tie_screw_length",
)


def _frozen(value):
    """A value as something a dictionary can key on: lists and the design's
    small records - a motor frame, a bolt size - as tuples of their fields."""
    if is_dataclass(value):
        return tuple(_frozen(v) for v in astuple(value))
    if hasattr(value, "model_dump"):                    # a pydantic record
        return tuple((k, _frozen(v)) for k, v in value.model_dump().items())
    if isinstance(value, list | tuple):
        return tuple(_frozen(v) for v in value)
    return value


def cad_key(spec: GearSpec) -> tuple:
    """Everything the solids are built from: the quick mesh's key, and more."""
    return (mesh_fingerprint(spec),
            tuple(_frozen(getattr(spec, name)) for name in SOLID_ONLY))


_CACHE: OrderedDict[tuple, Mesh] = OrderedDict()
_LOCK = threading.Lock()


def cached(key: tuple) -> Mesh | None:
    """The CAD mesh for ``key`` if one has been built, without building one.

    What the view asks first, so that coming back to a design - an undo, a
    preset tried and left - shows its solids at once rather than after a second
    build of the same thing.
    """
    with _LOCK:
        return _CACHE.get(key)


def cad_mesh(spec: GearSpec, quick: Mesh) -> Mesh:
    """The assembly's solids as a mesh with ``quick``'s parts.  Memoised.

    Seconds rather than milliseconds - building the solids is most of it - so
    the view calls this off its own thread, and only for a design that has
    stopped changing.  Safe to call from there: it shares nothing with the
    caller but this cache, which is locked.
    """
    key = cad_key(spec)
    with _LOCK:
        mesh = _CACHE.get(key)
        if mesh is not None:
            _CACHE.move_to_end(key)
            return mesh
    mesh = _build(spec, quick)
    with _LOCK:
        _CACHE[key] = mesh
        while len(_CACHE) > 3:
            _CACHE.popitem(last=False)
    return mesh


def _shapes(spec: GearSpec) -> dict:
    """Every part of the STEP assembly, placed, by name."""
    import cadquery as cq

    from ..export.solid import build_assembly

    placed = {}
    for child in build_assembly(spec).children:
        values = [v for v in child.obj.vals() if isinstance(v, cq.Shape)]
        if values:
            placed[child.name] = cq.Compound.makeCompound(values).moved(child.loc)
    return placed


def _tessellate(shape, angular: float = ANGULAR_TOLERANCE
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Every face of ``shape`` meshed: points, the surface normal at each, and
    triangles wound outward - each face on points of its own.

    What :meth:`cadquery.Shape.tessellate` does, and the normals besides: OCCT
    works them out from the surface itself at every node, so a flat face is
    flat to the last digit and a cylinder is a cylinder between its facets.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepLib import BRepLib_ToolTriangulatedShape
    from OCP.TopAbs import TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location

    shape.mesh(LINEAR_TOLERANCE, angular)
    points, normals, triangles = [], [], []
    offset = 0
    for face in shape.Faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, loc)
        if poly is None:
            continue
        if not poly.HasNormals():
            BRepLib_ToolTriangulatedShape.ComputeNormals_s(face.wrapped, poly)
        flip = -1.0 if face.wrapped.Orientation() == TopAbs_REVERSED else 1.0
        # One call a coordinate triple rather than three, and no transform
        # where there is none to apply: a quarter of a million nodes in a
        # rollered drive makes the per-node cost the whole of the wait.
        count = poly.NbNodes()
        xyz = np.array([poly.Node(i).Coord() for i in range(1, count + 1)])
        nrm = flip * np.array([poly.Normal(i).Coord() for i in range(1, count + 1)])
        if not loc.IsIdentity():
            move = loc.Transformation()
            turn = np.array([[move.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
            shift = np.array([move.Value(r, 4) for r in (1, 2, 3)])
            xyz = xyz @ turn.T + shift
            nrm = nrm @ turn.T
        points.append(xyz)
        normals.append(nrm)
        for t in poly.Triangles():
            a, b, c = (t.Value(k) + offset - 1 for k in (1, 2, 3))
            triangles.append((a, c, b) if flip < 0 else (a, b, c))
        offset += poly.NbNodes()
    if not points:
        return np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3), dtype=np.int64)
    return (np.vstack(points), np.vstack(normals),
            np.array(triangles, dtype=np.int64).reshape(-1, 3))


def _weld(vertices: np.ndarray, triangles: np.ndarray
          ) -> tuple[np.ndarray, np.ndarray]:
    """One shape as welded vertices and outward triangles."""
    # Each face of a solid is meshed with its own copy of the nodes on its
    # boundary.  They are the same points - the edge is discretised once and
    # shared - so they are welded back, which is what makes the part one
    # closed surface for the section to cap rather than a heap of faces.
    # By distance rather than by rounding: two copies a hair apart either side
    # of a rounding boundary would round apart and leave a slit.
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree

    pairs = cKDTree(vertices).query_pairs(1e-6, output_type="ndarray")
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(vertices), len(vertices)))
    _, label = connected_components(graph, directed=False)
    first = np.full(label.max() + 1, -1, dtype=np.int64)
    first[label[::-1]] = np.arange(len(vertices))[::-1]   # lowest index per group
    used, index = np.unique(first[label], return_inverse=True)
    welded = vertices[used]
    triangles = index.reshape(-1)[triangles]
    keep = ((triangles[:, 0] != triangles[:, 1])
            & (triangles[:, 1] != triangles[:, 2])
            & (triangles[:, 0] != triangles[:, 2]))
    triangles = triangles[keep]
    # Wound outward, as the quick mesh is: the divergence theorem's volume is
    # positive for a surface whose normals point out of it.
    a, b, c = (welded[triangles[:, k]] for k in range(3))
    if np.einsum("ij,ij->i", a, np.cross(b, c)).sum() < 0.0:
        triangles = triangles[:, ::-1]
    return welded, triangles


def _turn_to_local(vectors: np.ndarray, part: Part) -> np.ndarray:
    """The rotation half of :func:`_to_local`, for normals: they turn with the
    part and are not moved by its orbit."""
    angle = part.spin * part.phase
    c, s = math.cos(angle), math.sin(angle)
    out = vectors.copy()
    out[:, :2] = vectors[:, :2] @ np.array([[c, -s], [s, c]])
    return out


def _to_local(points: np.ndarray, part: Part, eccentricity: float) -> np.ndarray:
    """Undo the part's pose at crank zero: where the assembly put it, back to
    where the quick mesh builds it.  See :meth:`Mesh.world_vertices`."""
    xy = points[:, :2].copy()
    if part.orbits:
        xy -= (eccentricity * math.cos(part.phase),
               -eccentricity * math.sin(part.phase))
    angle = part.spin * part.phase
    c, s = math.cos(angle), math.sin(angle)
    out = points.copy()
    out[:, :2] = xy @ np.array([[c, -s], [s, c]])
    return out


def _build(spec: GearSpec, quick: Mesh) -> Mesh:
    shapes = _shapes(spec)
    blocks: list[np.ndarray] = []
    loops: list[tuple[np.ndarray, ...]] = []
    facet_part: list[int] = []
    parts: list[Part] = []
    shading: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    n = 0
    for index, part in enumerate(quick.parts):
        shape = shapes.get(part.name)
        if shape is None:
            # Nothing in the assembly by this name: draw what the quick mesh
            # drew, rather than a hole where a part should be.
            local = quick.vertices[part.vertices]
            offset = n - part.vertices.start
            part_loops = [tuple(lp + offset for lp in quick.loops[f])
                          for f in range(part.facets.start, part.facets.stop)]
        else:
            points, normals, faces = _tessellate(
                shape, BOUGHT_ANGULAR_TOLERANCE if part.group in BOUGHT_GROUPS
                else ANGULAR_TOLERANCE)
            vertices, triangles = _weld(points, faces)
            local = _to_local(vertices, part, quick.eccentricity)
            part_loops = [(tri,) for tri in (triangles + n).astype(np.int32)]
            shading[part.name] = (_to_local(points, part, quick.eccentricity),
                                  _turn_to_local(normals, part), faces,
                                  triangles)
        blocks.append(local)
        f0 = len(loops)
        loops.extend(part_loops)
        facet_part.extend([index] * len(part_loops))
        parts.append(replace(part, vertices=slice(n, n + len(local)),
                             facets=slice(f0, len(loops))))
        n += len(local)

    outer = [lp[0] for lp in loops]
    counts = np.array([len(o) for o in outer], dtype=np.int32)
    starts = np.zeros(len(counts), dtype=np.int32)
    np.cumsum(counts[:-1], out=starts[1:])
    return Mesh(
        vertices=np.vstack(blocks),
        loops=tuple(loops),
        facet_part=np.array(facet_part, dtype=np.int32),
        normal_ref=np.array([[o[0], o[len(o) // 3], o[2 * len(o) // 3]]
                             for o in outer], dtype=np.int32),
        outer_flat=np.concatenate(outer).astype(np.int32),
        outer_starts=starts,
        outer_counts=counts,
        parts=tuple(parts),
        eccentricity=quick.eccentricity,
        explode_span=quick.explode_span,
        frame_spin=quick.frame_spin,
        shading=shading,
    )
