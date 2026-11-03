"""Opt-in full-size immutable Field-map acceptance, not a synthetic room."""
import hashlib
import json
import os
from pathlib import Path
import time

import pytest


@pytest.mark.skipif(not os.environ.get("HORUS_ADAPTIVE_REAL_MAP"), reason="full saved Field fixture required")
def test_complete_saved_map_retains_fine_data_and_delivers_bounded_independent_views():
    from horus.mapping.adaptive_mesh import AdaptiveMeshStore, ensure_native
    from horus.mapping.mesh_protocol import decode_live_header
    ensure_native()
    directory = Path(os.environ["HORUS_ADAPTIVE_REAL_MAP"])
    files = sorted(directory.glob("*.hmsh"))
    assert len(files) == 642
    store = AdaptiveMeshStore()
    hashes = {}
    original_by_id = {}
    started = time.monotonic()
    for path in files:
        payload = path.read_bytes()
        hashes[path.name] = hashlib.sha256(payload).hexdigest()
        original_by_id[decode_live_header(payload).chunk_id] = hashes[path.name]
        assert store.ingest(payload), store.last_rejection
    assert len(store.pages) == 642
    assert sum(page.fine_triangles for page in store.pages.values()) == 3812282
    assert all(page.simplification_error_m <= .0150001 for page in store.pages.values())
    context, context_manifest, context_status = store.request_view("a" * 32, [], 2000000, 192 << 20, now=0.)
    region = {"id": "inspect", "shape": "box", "center": [0, 0, 0], "size": [2, 2, 2],
              "rotation": [0, 0, 0, 1], "priority": 1, "min_fraction": .25}
    selected, selected_manifest, status = store.request_view("b" * 32, [region], 1500000, 192 << 20, now=0.)
    assert len(context_manifest["pages"]) == len(selected_manifest["pages"]) == 642
    assert context_status["selected_triangles"] < 3812282
    assert context_status["over_budget"] is False
    assert status["selected_triangles"] <= 1500000
    assert status["over_budget"] is False
    assert status["regions_ready"] == 1
    assert {p["chunk_id"] for p in context_manifest["pages"]} == {p["chunk_id"] for p in selected_manifest["pages"]}
    assert len(context) == len(selected) == 643  # Epoch barrier plus complete spatial inventory.
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == hashes[path.name] for path in files)
    assert all(hashlib.sha256(page.fine).hexdigest() == original_by_id[page.chunk_id] for page in store.pages.values())
    print("\nREAL_MAP_RESULT=" + json.dumps({"elapsed_seconds": time.monotonic() - started,
        "pages": len(store.pages), "source_triangles": 3812282, "context": context_status,
        "adaptive": status, "all_original_sha256_unchanged": True}, sort_keys=True))
