"""Real vendored meshopt execution and independent, complete HMSH view contracts."""

from collections import Counter
import ctypes
import hashlib
import importlib.util
from pathlib import Path
import math
import shutil
import struct

import numpy as np
import pytest

from horus.mapping.adaptive_mesh import AdaptiveMeshStore, ensure_native, probe_saved_pages, _intersects, _regions
from horus.mapping.native.build_adaptive_mesh import cache_directory
from horus.mapping.mesh_protocol import (
    BlockGeometry, HMSH_V2_HEADER, HMSH_OPERATION_UPSERT, HMSH_OPERATION_DELETE,
    HMSH_OPERATION_CLEAR, decode_live_header, encode_live_payload,
)


A, B, C, D, E = (f"{i:032x}" for i in range(1, 6))
HUGE = 1 << 40


@pytest.fixture(scope="session", autouse=True)
def native_preflight():
    path = ensure_native()
    assert path.is_file() and path.suffix == ".so"
    assert path.parent == cache_directory()
    # Some user caches are on fuse/NTFS mounts that do not implement POSIX modes.
    assert path.stat().st_mode & 0o500 == 0o500
    return path


def payload(geometry=None, *, chunk=1, epoch=10, revision=1, operation=HMSH_OPERATION_UPSERT):
    return encode_live_payload(chunk_id=chunk, page_key=(chunk, 0, 0), epoch=epoch,
        revision=revision, operation=operation, geometry=geometry,
        max_payload_bytes=64 * 1024 * 1024)


def grid(*, n=32, offset=(0., 0., 0.), soup=False, colored=True, curved=False):
    vertices, colors, triangles = [], [], []
    for y in range(n + 1):
        for x in range(n + 1):
            xx, yy = offset[0] + x * .01, offset[1] + y * .01
            zz = offset[2] + (.02 * math.sin(x * .4) * math.cos(y * .3) if curved else 0.)
            vertices.append((xx, yy, zz))
            colors.append((64 + x * 3, 48 + y * 3, 128, 255))
    for y in range(n):
        for x in range(n):
            a = y * (n + 1) + x
            b, c, d = a + 1, a + n + 1, a + n + 2
            triangles.extend((a, b, d, a, d, c))
    if soup:
        colors = [colors[i] for i in triangles]
        vertices = [vertices[i] for i in triangles]
        triangles = list(range(len(triangles)))
    return BlockGeometry(tuple(vertices), tuple(colors) if colored else (), tuple(triangles))


def arrays(data):
    header = decode_live_header(data)
    stride = 16 if header.has_color else 12
    records = [data[HMSH_V2_HEADER.size + i * stride:HMSH_V2_HEADER.size + (i + 1) * stride]
               for i in range(header.vertex_count)]
    positions = np.ndarray((header.vertex_count, 3), dtype="<f4", buffer=data,
        offset=HMSH_V2_HEADER.size, strides=(stride, 4)).copy()
    indices = np.frombuffer(data, dtype="<u4", count=header.index_count,
        offset=HMSH_V2_HEADER.size + header.vertex_count * stride).reshape((-1, 3))
    return records, positions, indices


def boundary(data, *, geometric=False):
    records, _, indices = arrays(data)
    if geometric:
        records = [record[:12] for record in records]
    edges = Counter()
    for a, b, c in indices:
        if len({records[a], records[b], records[c]}) < 3:
            continue
        for u, v in ((a, b), (b, c), (c, a)):
            edges[tuple(sorted((records[u], records[v])))] += 1
    return {edge: count for edge, count in edges.items() if count != 2}


def sphere(identifier="a", center=(.16, .16, 0), diameter=.5, priority=1, fraction=.5):
    return {"id": identifier, "shape": "sphere", "center": list(center),
            "size": [diameter] * 3, "rotation": [0, 0, 0, 1],
            "priority": priority, "min_fraction": fraction}


def view(store, client=A, regions=None, triangles=HUGE, memory=HUGE, now=0.):
    return store.request_view(client, regions or [], triangles, memory, now=now)


def loaded(count=3):
    store = AdaptiveMeshStore()
    for i in range(count):
        assert store.ingest(payload(grid(offset=(i, 0., 0.)), chunk=i + 1))
    return store


@pytest.mark.parametrize("soup,colored", [(False, True), (True, True), (True, False)])
def test_real_meshopt_welds_without_touching_fine_and_preserves_every_boundary(soup, colored):
    original = payload(grid(soup=soup, colored=colored))
    store = AdaptiveMeshStore()
    assert store.ingest(original)
    page = store.pages[1]
    assert page.fine is original
    assert page.welded_vertices == 33 * 33
    assert page.boundary_vertices == 32 * 4
    assert 0 < page.coarse_triangles < page.fine_triangles / 2
    assert page.simplification_error_m <= .015
    assert boundary(page.coarse) == boundary(page.fine)
    assert set(arrays(page.coarse)[0]) <= set(arrays(page.fine)[0])
    # An open planar patch must cover exactly its original area, without holes or flips.
    for data in (page.fine, page.coarse):
        _, positions, triangles = arrays(data)
        cross = np.cross(positions[triangles[:, 1]] - positions[triangles[:, 0]],
                         positions[triangles[:, 2]] - positions[triangles[:, 0]])
        assert np.all(cross[:, 2] > 0)
        assert float(cross[:, 2].sum() / 2) == pytest.approx(.32 * .32, rel=1e-6)


def test_checked_in_obj_fixture_uses_real_kernel_and_original_wire_contract():
    fixture = Path(__file__).resolve().parents[2] / "contracts/fixtures/remote_scene/triangle.obj"
    vertices, indices = [], []
    for line in fixture.read_text().splitlines():
        parts = line.split()
        if parts and parts[0] == "v":
            vertices.append(tuple(map(float, parts[1:4])))
        elif parts and parts[0] == "f":
            indices.extend(int(value.split("/")[0]) - 1 for value in parts[1:])
    original = payload(BlockGeometry(tuple(vertices), ((255, 0, 0, 255),) * len(vertices), tuple(indices)))
    store = AdaptiveMeshStore()
    assert store.ingest(original)
    assert store.pages[1].fine == original
    assert boundary(store.pages[1].fine) == boundary(store.pages[1].coarse)
    updates, manifest, status = view(store, regions=[sphere(center=(.2, .2, 2), diameter=2)])
    data = next(update.payload for update in updates if update.operation == 0)
    assert data[:24] == original[:24] and data[40:] == original[40:]
    assert manifest["version"] == 1 and status["source_triangles"] == 1


def test_color_discontinuity_is_not_welded_or_averaged():
    left, right = grid(n=16), grid(n=16, offset=(.16, 0, 0))
    geometry = BlockGeometry(left.vertices + right.vertices,
        ((255, 0, 0, 255),) * len(left.vertices) + ((0, 0, 255, 255),) * len(right.vertices),
        left.triangles + tuple(index + len(left.vertices) for index in right.triangles))
    store = AdaptiveMeshStore()
    assert store.ingest(payload(geometry))
    page = store.pages[1]
    assert page.welded_vertices == len(geometry.vertices)
    assert boundary(page.coarse) == boundary(page.fine)
    assert {record[12:] for record in arrays(page.coarse)[0]} == {
        bytes([255, 0, 0, 255]), bytes([0, 0, 255, 255])}


def test_curved_source_respects_absolute_error_limit_and_component_inventory():
    store = AdaptiveMeshStore()
    geometry = grid(curved=True)
    # Include a disconnected small patch; it must not disappear to meet the target.
    island = grid(n=2, offset=(1., 1., .2))
    geometry = BlockGeometry(geometry.vertices + island.vertices, geometry.colors + island.colors,
        geometry.triangles + tuple(i + len(geometry.vertices) for i in island.triangles))
    assert store.ingest(payload(geometry))
    page = store.pages[1]
    assert page.simplification_error_m <= .015
    assert boundary(page.fine) == boundary(page.coarse)
    assert np.any(arrays(page.coarse)[1][:, 0] >= 1.)


@pytest.mark.parametrize("scale", [0., 1e-40, 3e38])
def test_degenerate_and_extreme_finite_coordinates_keep_original_safe(scale):
    geometry = BlockGeometry(((-scale, 0., 0.), (scale, 0., 0.), (0., scale, 0.)),
        ((128, 64, 0, 255),) * 3, (0, 1, 2))
    store = AdaptiveMeshStore()
    original = payload(geometry)
    assert store.ingest(original)
    assert store.pages[1].fine == original
    assert store.pages[1].fine_triangles == store.pages[1].coarse_triangles == 1
    assert store.pages[1].simplification_error_m == 0


def test_mixed_lod_neighbor_tiles_keep_identical_seam_geometry_and_colors():
    store = AdaptiveMeshStore()
    for chunk, offset in ((1, (0., 0., 0.)), (2, (.32, 0., 0.))):
        source = grid(offset=offset, soup=True)
        source = BlockGeometry(source.vertices, ((128, 96, 64, 255),) * len(source.vertices), source.triangles)
        assert store.ingest(payload(source, chunk=chunk))
    _, _, status = view(store, regions=[sphere(center=(.05, .16, 0), diameter=.12)])
    assert status["fine_pages"] == status["context_pages"] == 1
    fine, coarse = store.snapshot(A, {"request": "snapshot"})
    assert fine.payload[40:] == store.pages[1].fine[40:]
    def seam(data):
        return {edge for edge in boundary(data)
                if all(struct.unpack_from("<f", vertex)[0] == np.float32(.32) for vertex in edge)}
    assert seam(fine.payload) and seam(fine.payload) == seam(coarse.payload)


@pytest.mark.parametrize("change", [
    "short", "truncated", "trailing", "magic", "version", "color", "reserved",
    "count", "triangle_count", "index", "nan", "inf", "zero_revision",
    "chunk_outside", "invalid_operation", "delete_geometry", "clear_geometry",
])
def test_invalid_ingest_never_changes_previous_complete_state(change):
    store = loaded(1)
    original = store.pages[1]
    _, manifest, _ = view(store)
    data = bytearray(payload(grid(), revision=2))
    fields = list(HMSH_V2_HEADER.unpack_from(data))
    modifications = {"magic": (0, 0), "version": (1, 3), "color": (9, 2),
        "reserved": (10, 1), "count": (11, 10), "triangle_count": (12, 2),
        "zero_revision": (7, 0), "chunk_outside": (2, 0x40000000),
        "invalid_operation": (8, 3), "delete_geometry": (8, 1), "clear_geometry": (8, 2)}
    if change in modifications:
        index, value = modifications[change]
        fields[index] = value
        HMSH_V2_HEADER.pack_into(data, 0, *fields)
    elif change == "short": data = data[:12]
    elif change == "truncated": data = data[:-1]
    elif change == "trailing": data.extend(b"x")
    elif change == "index": struct.pack_into("<I", data, len(data) - 4, 0xffffffff)
    elif change in {"nan", "inf"}: struct.pack_into("<f", data, 52, float(change))
    assert not store.ingest(bytes(data))
    assert store.last_rejection
    assert store.pages[1] is original
    assert view(store)[1] == manifest


def test_stale_and_duplicate_source_revisions_epochs_and_delayed_clear():
    store = loaded(1)
    assert not store.ingest(store.pages[1].fine)
    assert not store.ingest(payload(grid(), epoch=9, revision=100))
    assert not store.ingest(payload(chunk=0, operation=2, epoch=10, revision=0))
    assert store.ingest(payload(grid(), epoch=10, revision=2))
    assert store.pages[1].source_revision == 2
    assert store.ingest(payload(chunk=0, operation=2, epoch=11, revision=0))
    assert not store.pages
    assert not store.ingest(payload(grid(), epoch=10, revision=999))
    assert store.ingest(payload(grid(), epoch=11, revision=1))


def test_cache_and_payload_limits_are_atomic_and_never_evict_context():
    original = payload(grid())
    store = AdaptiveMeshStore(max_cache_bytes=128)
    assert not store.ingest(original)
    assert not store.pages and store.source_epoch is None
    store = AdaptiveMeshStore(max_payload_bytes=128)
    assert not store.ingest(original)
    store = loaded(1)
    original = store.pages[1]
    store.max_cache_bytes = store.cached_bytes
    assert not store.ingest(payload(grid(), chunk=2))
    assert store.pages == {1: original}
    assert store.cached_bytes <= store.max_cache_bytes
    with pytest.raises(MemoryError, match="complete client view"):
        view(store)
    with pytest.raises(TypeError):
        store.pages[2] = original


def test_complete_context_no_frustum_drop_and_explicit_impossible_budget():
    store = loaded()
    updates, manifest, status = view(store, triangles=0, memory=0)
    assert {update.chunk_id for update in updates if update.operation == 0} == {1, 2, 3}
    assert len(manifest["pages"]) == 3
    assert status["context_pages"] == 3 and status["fine_pages"] == 0
    assert status["over_budget"] and not status["budget_feasible"]
    assert status["reason"] == "context_exceeds_budget"
    assert status["selected_triangles"] == status["context_triangles"] > 0
    assert status["selected_bytes"] == sum(len(store.pages[i].coarse) for i in store.pages)
    assert manifest["adaptive"] == status
    assert {"selected_triangles", "context_triangles", "fine_pages", "context_pages",
        "over_budget", "source_triangles", "selected_bytes", "regions_requested", "regions_ready"} <= status.keys()


def test_whole_fine_tiles_union_no_duplicate_surfaces_and_no_missing_context():
    store = loaded()
    regions = [sphere("near"), sphere("overlap"), sphere("far", center=(2.16, .16, 0))]
    updates, manifest, status = view(store, regions=regions)
    assert {update.chunk_id for update in updates if update.operation == 0} == {1, 2, 3}
    assert status["fine_pages"] == 2 and status["context_pages"] == 1
    assert status["regions_requested"] == status["regions_ready"] == 3
    assert len(manifest["pages"]) == 3
    for update in updates:
        if update.operation != 0:
            continue
        source = store.pages[update.chunk_id]
        expected = source.coarse if update.chunk_id == 2 else source.fine
        assert update.payload[:24] == expected[:24] and update.payload[40:] == expected[40:]


def test_oriented_box_uses_rotation_not_its_axis_aligned_envelope():
    store = AdaptiveMeshStore()
    for chunk, offset in enumerate(((.68, .68, 0), (.68, -.72, 0), (4., 4., 0)), 1):
        assert store.ingest(payload(grid(n=4, offset=offset), chunk=chunk))
    box = {"id": "diagonal", "shape": "box", "center": [0, 0, 0],
        "size": [3., .08, .2], "rotation": [0, 0, math.sin(math.pi/8), math.cos(math.pi/8)],
        "priority": 10, "min_fraction": .5}
    _, manifest, status = view(store, regions=[box])
    assert status["fine_pages"] == 1 and status["context_pages"] == 2
    assert status["regions"][0]["pages"] == 1
    assert len(manifest["pages"]) == 3
    selected = store.snapshot(A, {"request": "snapshot"})
    assert selected[0].payload[40:] == store.pages[1].fine[40:]
    assert selected[1].payload[40:] == store.pages[2].coarse[40:]


def test_fair_core_then_high_priority_outer_upgrades_use_incremental_costs():
    store = loaded(4)
    low = sphere("low", center=(.16, .16, 0), diameter=2.4, priority=0, fraction=.25)
    high = sphere("high", center=(3.16, .16, 0), diameter=2.4, priority=9, fraction=.25)
    baseline = sum(page.coarse_triangles for page in store.pages.values())
    cost = store.pages[1].fine_triangles - store.pages[1].coarse_triangles
    assert len({page.coarse_triangles for page in store.pages.values()}) == 1
    _, _, status = view(store, regions=[low, high], triangles=baseline + cost * 2)
    assert status["fine_pages"] == 2
    assert all(region["core_ready"] for region in status["regions"])
    assert all(region["fine_pages"] == 1 for region in status["regions"])
    assert status["selected_triangles"] <= baseline + cost * 2
    _, _, status = view(store, regions=[low, high], triangles=baseline + cost * 3, now=1)
    assert status["fine_pages"] == 3
    assert {region["id"]: region["fine_pages"] for region in status["regions"]} == {"low": 1, "high": 2}


def test_memory_budget_counts_increment_not_full_fine_plus_context():
    store = loaded(1)
    page = store.pages[1]
    _, _, status = view(store, regions=[sphere()], memory=len(page.fine))
    assert status["fine_pages"] == 1
    assert status["selected_bytes"] == len(page.fine)
    _, _, status = view(store, regions=[sphere()], memory=len(page.fine) - 1, now=1)
    assert status["fine_pages"] == 0 and status["over_budget"]
    assert status["granted_regions"] == [] and status["degraded"]


def test_minimum_cores_are_all_or_nothing_and_never_arbitrary_holey_tiles():
    store = loaded(2)
    regions = [sphere("one"), sphere("two", center=(1.16, .16, 0))]
    baseline = sum(page.coarse_triangles for page in store.pages.values())
    cost = store.pages[1].fine_triangles - store.pages[1].coarse_triangles
    updates, manifest, status = view(store, regions=regions, triangles=baseline + cost)
    assert status["over_budget"] and status["degraded"]
    assert status["reason"] == "minimum_cores_exceed_budget"
    assert status["granted_regions"] == [] and status["regions_ready"] == 0
    assert status["fine_pages"] == 0 and len(manifest["pages"]) == 2
    assert all(update.payload[40:] == store.pages[update.chunk_id].coarse[40:]
               for update in updates if update.operation == 0)


def test_every_granted_nested_volume_has_all_intersecting_fine_tiles_and_exact_union():
    store = loaded(5)
    box = {"id": "box", "shape": "box", "center": [3.16, .16, 0],
        "size": [2.5, 1., .3], "rotation": [0, 0, math.sin(.2), math.cos(.2)],
        "priority": 5, "min_fraction": .2}
    regions = [sphere("sphere", center=(1.16, .16, 0), diameter=4., fraction=.15), box]
    baseline = sum(page.coarse_triangles for page in store.pages.values())
    cost = max(page.fine_triangles - page.coarse_triangles for page in store.pages.values())
    for step in range(6):
        _, manifest, status = view(store, regions=regions, triangles=baseline + cost * step, now=step)
        fine_union = set()
        requested = {region["id"]: region for region in regions}
        for grant in status["granted_regions"]:
            assert requested[grant["id"]]["min_fraction"] <= grant["fraction"] <= 1
            assert grant["center"] == requested[grant["id"]]["center"]
            assert grant["size"] == pytest.approx(np.asarray(requested[grant["id"]]["size"]) * grant["fraction"])
            parsed = _regions([{**grant, "priority": 0, "min_fraction": 0}])[0]
            fine_union.update(chunk for chunk, page in store.pages.items() if _intersects(parsed, page.bounds))
        assert status["fine_pages"] == len(fine_union)
        assert status["context_pages"] + status["fine_pages"] == len(store.pages)
        inventory = store.snapshot(A, {"request": "snapshot"})
        assert {update.chunk_id for update in inventory} == set(store.pages)
        for update in inventory:
            page = store.pages[update.chunk_id]
            expected = page.fine if update.chunk_id in fine_union else page.coarse
            assert update.payload[40:] == expected[40:]
        assert status["regions_ready"] == sum(grant["fraction"] == 1 for grant in status["granted_regions"])
        assert manifest["adaptive"]["granted_regions"] == status["granted_regions"]
        if not status["over_budget"]:
            assert status["selected_triangles"] <= baseline + cost * step


def test_sparse_and_zero_chunk_ids_are_legal_and_repaired_by_membership():
    store = AdaptiveMeshStore(max_pages=2)
    assert store.ingest(payload(grid(), chunk=123456))
    assert store.ingest(payload(grid(offset=(1., 0., 0.)), chunk=0))
    assert set(store.pages) == {0, 123456}
    assert not store.ingest(payload(grid(), chunk=999999))
    _, manifest, _ = view(store)
    repaired = store.snapshot(A, {"request": "pages", "epoch": manifest["epoch"], "ids": [123456, 0]})
    assert [update.chunk_id for update in repaired] == [0, 123456]
    assert all(update.operation == 0 for update in repaired)
    assert store.snapshot(A, {"request": "pages", "ids": [999999]}) == []
    assert store.ingest(payload(chunk=123456, revision=2, operation=1))
    deleted, _, _ = view(store, now=1)
    assert len(deleted) == 1 and deleted[0].chunk_id == 123456 and deleted[0].operation == 1
    assert store.snapshot(A, {"request": "pages", "ids": [123456]}) == deleted


def test_live_plus_tombstone_inventory_capacity_rejects_third_identity_atomically():
    store = AdaptiveMeshStore(max_pages=2)
    assert store.ingest(payload(grid(), chunk=123456))
    assert store.ingest(payload(grid(offset=(1., 0., 0.)), chunk=0))
    assert store.ingest(payload(chunk=123456, revision=2, operation=1))
    _, before, _ = view(store)
    previous_page = store.pages[0]
    previous_bytes = store.cached_bytes
    assert not store.ingest(payload(grid(), chunk=999999))
    assert "inventory capacity" in store.last_rejection
    assert store.pages == {0: previous_page}
    assert store.cached_bytes == previous_bytes
    _, after, _ = view(store)
    assert before == after
    assert len(after["pages"]) == 2
    assert store.snapshot(A, {"request": "pages", "ids": [123456]})[0].operation == 1


def test_packaged_vendor_is_byte_identical_to_pinned_existing_sources():
    root = Path(__file__).resolve().parents[2]
    packaged = root / "python/horus/mapping/native/vendor"
    existing = root / "rust/native/core/vendor/meshoptimizer"
    for name in ("meshoptimizer.h", "simplifier.cpp", "allocator.cpp", "indexgenerator.cpp", "LICENSE.md", "PROVENANCE.md"):
        assert hashlib.sha256((packaged / name).read_bytes()).digest() == hashlib.sha256((existing / name).read_bytes()).digest()


def test_build_from_readonly_installed_layout_needs_no_repo_or_site_package_writes(tmp_path, monkeypatch):
    original = Path(__file__).resolve().parents[1] / "horus/mapping/native"
    installed = tmp_path / "site-packages/horus/mapping/native"
    installed.mkdir(parents=True)
    for name in ("adaptive_mesh.cpp", "build_adaptive_mesh.py"):
        shutil.copyfile(original / name, installed / name)
    shutil.copytree(original / "vendor", installed / "vendor")
    for path in installed.rglob("*"):
        path.chmod(0o555 if path.is_dir() else 0o444)
    installed.chmod(0o555)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    spec = importlib.util.spec_from_file_location("installed_adaptive_mesh_build", installed / "build_adaptive_mesh.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    library = module.build_library()
    assert library.is_relative_to(tmp_path / "cache/horus/adaptive_mesh")
    assert library.stat().st_mode & 0o777 == 0o755
    assert not list(installed.rglob("*.so"))


def test_independent_clients_monotonic_lod_revisions_and_exact_repairs():
    store = loaded()
    updates_a, manifest_a, _ = view(store, regions=[sphere()])
    updates_b, manifest_b, _ = view(store, client=B, regions=[sphere(center=(2.16, .16, 0))])
    assert manifest_a["epoch"] != manifest_b["epoch"]
    assert next(u for u in updates_a if u.chunk_id == 1).payload[40:] == store.pages[1].fine[40:]
    assert next(u for u in updates_b if u.chunk_id == 1).payload[40:] == store.pages[1].coarse[40:]
    assert view(store, regions=[sphere()])[0] == []
    swapped, swapped_manifest, _ = view(store, regions=[sphere(center=(2.16, .16, 0))], now=1)
    assert {u.chunk_id for u in swapped} == {1, 3}
    old_revisions = {u.chunk_id: u.revision for u in updates_a if u.operation == 0}
    assert all(u.revision > old_revisions[u.chunk_id] for u in swapped)
    assert swapped_manifest["epoch"] == manifest_a["epoch"]
    assert store.snapshot(B, {"request": "snapshot"}) == [u for u in updates_b if u.operation == 0]
    assert store.snapshot(A, {"request": "pages", "epoch": swapped_manifest["epoch"], "ids": [3, 1, 3]}) == sorted(swapped, key=lambda u: u.chunk_id)
    repaired = store.snapshot(A, {"request": "snapshot", "epoch": "0"})
    assert repaired[0].operation == 2
    assert len(repaired) == 4


def test_source_change_delete_slot_reuse_reset_and_last_committed_repair():
    store = loaded(1)
    first, manifest, _ = view(store, regions=[sphere()])
    previous = next(u for u in first if u.operation == 0)
    assert store.ingest(payload(grid(offset=(.01, 0, 0)), revision=2))
    # Repairs remain a coherent, immutable view until request_view commits another one.
    assert store.snapshot(A, {"request": "snapshot"}) == [previous]
    changed, _, _ = view(store, regions=[sphere()], now=1)
    assert len(changed) == 1 and changed[0].revision > previous.revision
    assert store.ingest(payload(operation=1, revision=3))
    deleted, deleted_manifest, status = view(store, now=2)
    assert len(deleted) == 1 and deleted[0].operation == 1
    assert status["source_triangles"] == status["selected_triangles"] == 0
    assert deleted_manifest["pages"] == [{"chunk_id": 1, "revision": str(deleted[0].revision), "operation": 1}]
    assert store.snapshot(A, {"request": "pages", "epoch": manifest["epoch"], "ids": [1]}) == deleted
    assert store.ingest(payload(grid(), revision=4))
    reused, _, _ = view(store, now=3)
    assert reused[0].revision > deleted[0].revision
    assert store.ingest(payload(chunk=0, epoch=11, revision=0, operation=2))
    assert store.ingest(payload(grid(), epoch=11, revision=1))
    reset, new_manifest, _ = view(store, now=4)
    assert [u.operation for u in reset] == [2, 0]
    assert int(new_manifest["epoch"]) > int(manifest["epoch"])
    assert reset[1].revision > reused[0].revision
    assert all(u.epoch == int(new_manifest["epoch"]) for u in reset)


def test_four_clients_ttl_and_rejoin_use_new_epochs_without_evicting_active_views():
    store = loaded(1)
    first_epochs = {}
    for client in (A, B, C, D):
        _, manifest, _ = view(store, client=client)
        first_epochs[client] = int(manifest["epoch"])
    with pytest.raises(ValueError, match="four"):
        view(store, client=E, now=1)
    view(store, client=A, now=10)
    assert store.expire_clients(15) == [B, C, D]
    assert store.snapshot(B, {"request": "snapshot"}) == []
    _, manifest, _ = view(store, client=B, now=15)
    assert int(manifest["epoch"]) > first_epochs[B]
    assert store.expire_clients(25) == [A]
    assert len(store.pages) == 1


@pytest.mark.parametrize("bad", [None, {}, {"request": "view"}, {"request": []},
    {"request": "pages", "ids": [True]}, {"request": "pages", "ids": [-1]},
    {"request": "pages", "ids": [0x40000000]}])
def test_malformed_repair_cannot_amplify_to_full_map(bad):
    store = loaded()
    view(store)
    assert store.snapshot(A, bad) == []


@pytest.mark.parametrize("change", ["nan", "size", "rotation", "duplicate", "fraction", "priority"])
def test_bad_regions_are_rejected_before_committing_view(change):
    store = loaded(1)
    first = view(store)[1]
    region = sphere()
    if change == "nan": region["center"][0] = math.nan
    if change == "size": region["size"][0] = -1
    if change == "rotation": region["rotation"] = [0] * 4
    if change == "fraction": region["min_fraction"] = 2
    if change == "priority": region["priority"] = math.inf
    regions = [region, region] if change == "duplicate" else [region]
    with pytest.raises(ValueError):
        view(store, regions=regions)
    assert view(store)[1] == first


def test_bad_clock_client_budget_and_returned_metadata_cannot_corrupt_store():
    store = loaded(1)
    with pytest.raises(ValueError): view(store, client="not-a-hex-id")
    with pytest.raises(ValueError): view(store, triangles=True)
    with pytest.raises(ValueError): view(store, memory=-1)
    with pytest.raises(ValueError): view(store, now=math.nan)
    _, manifest, status = view(store, regions=[sphere()])
    manifest["adaptive"]["regions"][0]["ready"] = False
    status["regions"][0]["id"] = "changed"
    manifest["adaptive"]["granted_regions"][0]["size"][0] = -99
    status["granted_regions"][0]["center"][0] = -99
    _, again, _ = view(store, regions=[sphere()])
    assert again["adaptive"]["regions"][0]["ready"]
    assert again["adaptive"]["regions"][0]["id"] == "a"
    assert again["adaptive"]["granted_regions"][0]["size"][0] > 0
    assert again["adaptive"]["granted_regions"][0]["center"][0] == .16


def test_zero_area_only_components_do_not_force_whole_valid_patch_fallback():
    original = grid()
    vertices = original.vertices + ((8., 8., 8.), (9., 9., 9.), (10., 10., 10.), (11., 11., 11.))
    a = len(original.vertices)
    source = BlockGeometry(vertices, original.colors + ((128, 96, 64, 255),) * 4,
        original.triangles + (a, a, a, a + 1, a + 2, a + 3))
    data = payload(source)
    store = AdaptiveMeshStore()
    assert store.ingest(data)
    page = store.pages[1]
    assert page.fine == data
    assert page.degenerate_triangles == 2
    assert not page.native_fallback_reason
    assert page.coarse_triangles < len(original.triangles) // 6
    assert boundary(page.coarse) == boundary(payload(original))
    fields, positions = store._decode(data)
    baseline = store._derive(data, fields, positions, baseline=True)
    assert baseline.native_fallback_reason == "component_lost"
    assert baseline.coarse_triangles == baseline.fine_triangles


@pytest.mark.parametrize("thickness", [1e-6, 1e-20, 1e-40])
def test_zero_area_filter_preserves_arbitrarily_thin_nonzero_triangles(thickness):
    source = BlockGeometry(((0., 0., 0.), (1., 0., 0.), (1., thickness, 0.)),
        ((128, 96, 64, 255),) * 3, (0, 1, 2))
    original = payload(source)
    store = AdaptiveMeshStore()
    assert store.ingest(original)
    page = store.pages[1]
    assert page.degenerate_triangles == 0
    assert page.coarse_triangles == page.fine_triangles == 1
    assert boundary(page.coarse) == boundary(original)


def test_offline_probe_reports_baseline_component_fallback_without_mutating_fixture(tmp_path):
    source = grid()
    i = len(source.vertices)
    source = BlockGeometry(source.vertices + ((1., 1., 1.),),
        source.colors + ((128, 96, 64, 255),), source.triangles + (i, i, i))
    data = payload(source)
    path = tmp_path / "page_1.hmsh"
    path.write_bytes(data)
    result = probe_saved_pages(tmp_path)
    assert result["before_triangles"] == result["source_triangles"]
    assert result["before_fallbacks"] == {"component_lost": 1}
    assert result["fallbacks"] == {}
    assert result["coarse_triangles"] < result["source_triangles"] / 2
    assert result["zero_area_faces"] == 1
    assert path.read_bytes() == data


def test_lost_closed_component_is_restored_without_reverting_healthy_component():
    patch = grid()
    n = len(patch.vertices)
    tetrahedron = ((1., 1., 0.), (1.0001, 1., 0.), (1., 1.0001, 0.), (1., 1., .0001))
    tetra_faces = (0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3)
    source = BlockGeometry(patch.vertices + tetrahedron,
        patch.colors + ((128, 96, 64, 255),) * 4,
        patch.triangles + tuple(n + index for index in tetra_faces))
    data = payload(source)
    store = AdaptiveMeshStore()
    assert store.ingest(data)
    page = store.pages[1]
    assert page.fine == data
    assert page.native_fallback_reason == "unsafe_components_restored"
    assert page.source_components == 2 and page.restored_components == 1
    assert page.restored_triangles == page.largest_restored_component == 4
    assert page.coarse_triangles == page.native_simplified_triangles + 4
    assert page.coarse_triangles < page.fine_triangles / 2
    assert boundary(page.coarse) == boundary(data)
    original_records, original_positions, original_indices = arrays(data)
    coarse_records, coarse_positions, coarse_indices = arrays(page.coarse)
    def island_faces(records, positions, indices):
        return Counter(tuple(records[i] for i in face) for face in indices
                       if all(positions[i, 0] >= 1. for i in face))
    assert island_faces(coarse_records, coarse_positions, coarse_indices) == island_faces(
        original_records, original_positions, original_indices)


@pytest.mark.parametrize("missing_component", [False, True])
def test_native_boundary_failure_restores_only_unsafe_component(missing_component):
    patch = grid(n=8)
    store = AdaptiveMeshStore()
    assert store.ingest(payload(patch))
    source_records = arrays(store.pages[1].fine)[0]
    compact_records, _, compact_indices = arrays(store.pages[1].coarse)
    original_index = {record: i for i, record in enumerate(source_records)}
    healthy = [original_index[compact_records[i]] for i in compact_indices.reshape(-1)]
    n = len(patch.vertices)
    square = (n, n + 1, n + 2, n, n + 2, n + 3)
    before = np.array((*patch.triangles, *square), dtype=np.uint32)
    # A deliberately unsafe candidate has either lost one square face or the whole square.
    candidate = np.array(healthy if missing_component else healthy + list(square[:3]), dtype=np.uint32)
    output = np.empty(len(before) + len(candidate), dtype=np.uint32)
    output_count = ctypes.c_size_t()
    diagnostics = (ctypes.c_size_t * 16)()
    restore = store._native.horus_adaptive_mesh_restore_components
    restore.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t,
        ctypes.c_size_t, ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
    restore.restype = ctypes.c_int
    assert restore(before.ctypes.data, len(before), candidate.ctypes.data, len(candidate), n + 4,
        output.ctypes.data, ctypes.byref(output_count), diagnostics) == 0
    restored = output[:output_count.value].tolist()
    assert restored == healthy + list(square)
    assert diagnostics[7] == 2 and diagnostics[8] == 1
    assert diagnostics[9] == diagnostics[10] == 2
    assert diagnostics[11] == 1 and diagnostics[12] == int(missing_component)


def test_outer_border_rgba_and_internal_color_discontinuity_are_preserved():
    left, right = grid(n=16), grid(n=16, offset=(.16, 0., 0.))
    source = BlockGeometry(left.vertices + right.vertices,
        ((255, 0, 0, 255),) * len(left.vertices) + ((0, 0, 255, 255),) * len(right.vertices),
        left.triangles + tuple(i + len(left.vertices) for i in right.triangles))
    data = payload(source)
    store = AdaptiveMeshStore()
    assert store.ingest(data)
    page = store.pages[1]
    assert page.fine == data
    assert boundary(page.coarse) == boundary(data)
    assert boundary(page.coarse, geometric=True) == boundary(data, geometric=True)
    source_records = arrays(data)[0]
    coarse_records, _, coarse_indices = arrays(page.coarse)
    border_positions = {position for edge in boundary(data, geometric=True) for position in edge}
    assert {record for record in source_records if record[:12] in border_positions} <= set(coarse_records)
    assert {record[12:] for record in coarse_records} == {bytes([255, 0, 0, 255]), bytes([0, 0, 255, 255])}
    assert all(len({coarse_records[index][12:] for index in face}) == 1 for face in coarse_indices)
    assert page.simplification_error_m <= .015


def test_coincident_double_sided_thin_patch_keeps_both_colours_and_full_area():
    patch = grid(n=8)
    n = len(patch.vertices)
    reversed_faces = tuple(n + patch.triangles[i + j] for i in range(0, len(patch.triangles), 3)
                           for j in (0, 2, 1))
    source = BlockGeometry(patch.vertices + patch.vertices,
        ((255, 0, 0, 255),) * n + ((0, 0, 255, 255),) * n,
        patch.triangles + reversed_faces)
    data = payload(source)
    store = AdaptiveMeshStore()
    assert store.ingest(data)
    page = store.pages[1]
    assert page.fine == data
    assert boundary(page.coarse) == boundary(data)
    records, positions, indices = arrays(page.coarse)
    areas = Counter()
    for face in indices:
        colours = {records[i][12:] for i in face}
        assert len(colours) == 1
        a, b, c = positions[face]
        areas[next(iter(colours))] += float(np.linalg.norm(np.cross(b - a, c - a))) * .5
    assert set(areas) == {bytes([255, 0, 0, 255]), bytes([0, 0, 255, 255])}
    assert all(area == pytest.approx(.08 * .08, rel=1e-6) for area in areas.values())
