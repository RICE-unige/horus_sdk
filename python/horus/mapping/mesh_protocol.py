"""Source-independent HMSH v2 wire types and full-geometry encoder."""

from array import array
from dataclasses import dataclass
import struct
import sys

HMSH_MAGIC = 0x484D5348
HMSH_VERSION_LIVE = 2
HMSH_OPERATION_UPSERT = 0
HMSH_OPERATION_DELETE = 1
HMSH_OPERATION_CLEAR = 2
HMSH_V2_HEADER = struct.Struct("<IIIiiiQQBBHII")
FLOAT32_MAX = struct.unpack("<f", bytes.fromhex("ffff7f7f"))[0]

BlockKey = tuple[int, int, int]


@dataclass(frozen=True)
class BlockGeometry:
    vertices: tuple[tuple[float, float, float], ...]
    colors: tuple[tuple[int, int, int, int], ...]
    triangles: tuple[int, ...]


@dataclass(frozen=True)
class MeshPageUpdate:
    chunk_id: int
    page_key: BlockKey
    epoch: int
    revision: int
    operation: int
    payload: bytes
    vertex_count: int
    triangle_count: int


@dataclass(frozen=True)
class LiveHeader:
    chunk_id: int
    page_key: BlockKey
    epoch: int
    revision: int
    operation: int
    has_color: bool
    vertex_count: int
    index_count: int


def decode_live_header(payload: bytes) -> LiveHeader:
    if len(payload) < HMSH_V2_HEADER.size:
        raise ValueError("HMSH v2 payload is shorter than its header")
    (
        magic,
        version,
        chunk_id,
        page_x,
        page_y,
        page_z,
        epoch,
        revision,
        operation,
        has_color,
        _reserved,
        vertex_count,
        index_count,
    ) = HMSH_V2_HEADER.unpack_from(payload)
    if magic != HMSH_MAGIC or version != HMSH_VERSION_LIVE:
        raise ValueError("payload is not HMSH v2")
    return LiveHeader(
        chunk_id=chunk_id,
        page_key=(page_x, page_y, page_z),
        epoch=epoch,
        revision=revision,
        operation=operation,
        has_color=bool(has_color),
        vertex_count=vertex_count,
        index_count=index_count,
    )


def encode_live_payload(
    *,
    chunk_id: int,
    page_key: BlockKey,
    epoch: int,
    revision: int,
    operation: int,
    geometry: BlockGeometry | None,
    max_payload_bytes: int,
) -> bytes:
    if operation not in {
        HMSH_OPERATION_UPSERT,
        HMSH_OPERATION_DELETE,
        HMSH_OPERATION_CLEAR,
    }:
        raise ValueError(f"invalid HMSH operation: {operation}")

    vertex_count = len(geometry.vertices) if geometry is not None else 0
    index_count = len(geometry.triangles) if geometry is not None else 0
    has_color = 1 if geometry is not None and len(geometry.colors) == vertex_count else 0
    payload_size = HMSH_V2_HEADER.size + vertex_count * (16 if has_color else 12) + index_count * 4
    if payload_size > max_payload_bytes:
        raise ValueError(
            f"HMSH page {page_key} is {payload_size / (1024 * 1024):.1f} MiB; "
            "split the source into smaller pages without reducing map quality"
        )
    header = HMSH_V2_HEADER.pack(
        HMSH_MAGIC,
        HMSH_VERSION_LIVE,
        int(chunk_id) & 0xFFFFFFFF,
        int(page_key[0]),
        int(page_key[1]),
        int(page_key[2]),
        int(epoch) & 0xFFFFFFFFFFFFFFFF,
        int(revision) & 0xFFFFFFFFFFFFFFFF,
        int(operation),
        has_color,
        0,
        vertex_count,
        index_count,
    )
    if geometry is None:
        return header

    payload = bytearray(header)
    if has_color:
        for vertex, color in zip(geometry.vertices, geometry.colors):
            payload.extend(struct.pack("<fffBBBB", *vertex, *color))
    else:
        for vertex in geometry.vertices:
            payload.extend(struct.pack("<fff", *vertex))

    indices = array("I", geometry.triangles)
    if sys.byteorder != "little":
        indices.byteswap()
    payload.extend(indices.tobytes())
    return bytes(payload)
