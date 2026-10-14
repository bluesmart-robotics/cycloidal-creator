"""The 3D view's CAD mesh: the exported solids, in the quick mesh's frames.

The quick mesh is held to the solids by volume in ``tests/test_viz.py``; this
one *is* the solids, so what is worth holding is that nothing was lost on the
way - every part closed, every part where the quick mesh puts it at every crank
angle, and a cache that cannot serve one design's solids for another.
"""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest

from cycloidgen.core.spec import OutputMember, preset
from cycloidgen.viz import cadmesh
from cycloidgen.viz.mesh import mesh_for_spec

ROOT = Path(__file__).resolve().parent.parent


def _triangles(mesh, part) -> np.ndarray:
    return np.array([mesh.loops[f][0] for f in range(part.facets.start,
                                                      part.facets.stop)])


@pytest.fixture(scope="module", params=["carrier", "ring"])
def pair(request):
    spec = preset(21)
    if request.param == "ring":
        spec.output_member = OutputMember.RING
    quick = mesh_for_spec(spec)
    return spec, quick, cadmesh.cad_mesh(spec, quick)


def test_every_part_is_a_closed_surface_enclosing_its_solid(pair):
    """Welded where the faces meet, wound outward, and the volume of the part.

    Closed is what the section needs to cap a part rather than open it, and a
    triangle wound the wrong way would show as a negative contribution to the
    volume long before anyone noticed it shaded black.
    """
    spec, _, mesh = pair
    solids = cadmesh._shapes(spec)
    v = mesh.vertices
    for part in mesh.parts:
        tris = _triangles(mesh, part)
        edges = np.sort(np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]],
                                        tris[:, [2, 0]]]), axis=1)
        _, uses = np.unique(edges, axis=0, return_counts=True)
        assert (uses == 2).all(), f"{part.name} is not closed"
        volume = np.einsum("ij,ij->i", v[tris[:, 0]],
                           np.cross(v[tris[:, 1]], v[tris[:, 2]])).sum() / 6.0
        # The bought parts are tessellated coarser on purpose - see
        # `cadmesh.BOUGHT_ANGULAR_TOLERANCE` - and their balls and needles
        # lose a little more to it.
        coarse = part.group in cadmesh.BOUGHT_GROUPS
        assert volume == pytest.approx(solids[part.name].Volume(),
                                       rel=0.02 if coarse else 0.005), part.name


@pytest.mark.parametrize("crank", [0.0, 0.7, 2.9])
def test_every_part_is_where_the_quick_mesh_puts_it(pair, crank):
    """Same parts, same motions, same places - at any angle, not just at zero.

    Compared by bounding box, which the facets move by a few hundredths and a
    part put in the wrong frame moves by the eccentricity or more.
    """
    _, quick, mesh = pair
    assert [p.name for p in mesh.parts] == [p.name for p in quick.parts]
    for explode in (0.0, 0.6):
        a = quick.world_vertices(crank, explode)
        b = mesh.world_vertices(crank, explode)
        for p, q in zip(mesh.parts, quick.parts, strict=True):
            lo = np.abs(b[p.vertices].min(axis=0) - a[q.vertices].min(axis=0))
            hi = np.abs(b[p.vertices].max(axis=0) - a[q.vertices].max(axis=0))
            assert max(lo.max(), hi.max()) < 0.1, (p.name, crank, explode)


def test_the_key_covers_everything_the_solids_read():
    """A field the solids read and the key does not is a stale model on screen.

    Read out of the source, the same way the explanations are held to the checks:
    every ``spec.<name>`` in the solid builder has to be in the quick mesh's
    fingerprint or in :data:`cadmesh.SOLID_ONLY`.
    """
    solid = (ROOT / "cycloidgen" / "export" / "solid.py").read_text(encoding="utf-8")
    mesh = (ROOT / "cycloidgen" / "viz" / "mesh.py").read_text(encoding="utf-8")
    fingerprint = mesh[mesh.index("def mesh_fingerprint"):mesh.index("_CACHE:")]
    read = set(re.findall(r"spec\.([a-z_][a-z0-9_]*)", solid))
    keyed = set(re.findall(r"spec\.([a-z_][a-z0-9_]*)", fingerprint))
    assert not read - keyed - set(cadmesh.SOLID_ONLY)


def test_the_key_moves_with_a_field_only_the_solids_read():
    spec = preset(21)
    before = cadmesh.cad_key(spec)
    assert cadmesh.cad_key(spec) == before
    spec.edge_chamfer = 0.8
    assert cadmesh.cad_key(spec) != before
    assert mesh_for_spec(spec) is mesh_for_spec(preset(21))     # quick: unmoved


def test_a_built_mesh_is_served_again_and_looked_up_without_building():
    spec = preset(15)
    key = cadmesh.cad_key(spec)
    built = cadmesh.cad_mesh(spec, mesh_for_spec(spec))
    assert cadmesh.cached(key) is built
    assert cadmesh.cad_mesh(spec, mesh_for_spec(spec)) is built
    other = preset(15)
    other.edge_chamfer = 0.2
    assert cadmesh.cached(cadmesh.cad_key(other)) is None


def test_it_is_finer_than_the_quick_mesh_where_it_matters():
    """The point of it: a ring pin's facets, which the quick mesh caps at 24."""
    from cycloidgen.analysis.bearings import pin_diameters

    spec = preset(21)
    mesh = cadmesh.cad_mesh(spec, mesh_for_spec(spec))
    pins = next(p for p in mesh.parts if p.name == "ring_pins")
    v = mesh.vertices[pins.vertices]
    radius = pin_diameters(spec)[0] / 2.0
    offset = v[:, :2] - (spec.pin_circle_radius, 0.0)      # the first pin
    wall = np.abs(np.hypot(*offset.T) - radius) < 0.02
    angles = np.unique(np.round(np.degrees(np.arctan2(offset[wall, 1],
                                                      offset[wall, 0])), 2))
    assert len(angles) >= 36
