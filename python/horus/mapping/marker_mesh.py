"""Standard ROS TRIANGLE_LIST ingestion into source-independent live map pages."""

import copy
import math

from .live_mesh import LiveMeshState
from .mesh_protocol import BlockGeometry, FLOAT32_MAX


def point_transform(translation, rotation):
    x, y, z, w = rotation
    length = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("mesh pose must have a finite, nonzero quaternion")
    x, y, z, w = (v / length for v in rotation)
    xx, xy, xz = 1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)
    yx, yy, yz = 2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)
    zx, zy, zz = 2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)
    tx, ty, tz = translation
    return lambda p: (xx*p[0]+xy*p[1]+xz*p[2]+tx,
                      yx*p[0]+yy*p[1]+yz*p[2]+ty,
                      zx*p[0]+zy*p[1]+zz*p[2]+tz)


def vector(value):
    return tuple(float(getattr(value, axis)) for axis in ("x", "y", "z"))


def quaternion(value):
    return tuple(float(getattr(value, axis)) for axis in ("x", "y", "z", "w"))


def rgba(value):
    channels = tuple(float(getattr(value, axis)) for axis in ("r", "g", "b", "a"))
    if not all(math.isfinite(v) and 0 <= v <= 1 for v in channels):
        raise ValueError("mesh colors must be finite and in [0, 1]")
    return tuple(round(v * 255) for v in channels)


class MarkerMeshState:
    """One source topic, many namespaces/IDs; source omissions never mean delete.

    `resolve_transform(frame, stamp)` returns translation and quaternion from
    that frame to `state.frame_id`. Fixed-frame messages need no TF lookup.
    Frame-locked markers are deliberately rejected, not frozen at a wrong pose.
    """

    def __init__(self, *, triangles_per_page=4096, max_input_points=3000000, **limits):
        if not 1 <= triangles_per_page <= 20000 or max_input_points < 3:
            raise ValueError("invalid mesh page/input limits")
        self.state = LiveMeshState(**limits)
        self.triangles_per_page = triangles_per_page
        self.max_input_points = max_input_points
        self._parts = {}
        self._expires = {}

    def apply(self, markers, *, now, resolve_transform=None):
        # Validate an entire MarkerArray before changing the authoritative map.
        staged = copy.copy(self)
        staged.state = copy.copy(self.state)
        for name in ("_keys", "_current", "_revisions"):
            setattr(staged.state, name, getattr(self.state, name).copy())
        staged.state._free = self.state._free.copy()
        staged._parts = self._parts.copy()
        staged._expires = self._expires.copy()
        updates = []
        # A complete MarkerArray snapshot often starts with DELETEALL. Compare its
        # final contents with the cache instead of clearing/reuploading every page.
        clears = [i for i, marker in enumerate(markers) if marker.action == 3]
        if clears and clears[-1] < len(markers) - 1:
            markers = markers[clears[-1] + 1:]
            present = {(str(m.ns), int(m.id)) for m in markers if m.action == 0}
            for key in list(staged._parts):
                if key not in present:
                    updates.extend(staged._delete(key))
        for marker in markers:
            updates.extend(staged._apply_one(marker, now, resolve_transform))
        self.__dict__.update(staged.__dict__)
        return updates

    def _delete(self, key):
        updates = self.state.replace_many(
            ((key, part), None) for part in range(self._parts.get(key, 0)))
        self._parts.pop(key, None)
        self._expires.pop(key, None)
        return updates

    def _apply_one(self, marker, now, resolve_transform):
        key = (str(marker.ns), int(marker.id))
        if marker.action == 3:
            self._parts.clear()
            self._expires.clear()
            return [self.state.clear()]
        if marker.action == 2:
            return self._delete(key)
        if marker.action != 0 or marker.type != 11:
            raise ValueError("live map adapter accepts TRIANGLE_LIST ADD, DELETE and DELETEALL only")
        if marker.frame_locked:
            raise ValueError("frame_locked markers need a dynamic visualizer, not a fixed-frame map cache")
        count = len(marker.points)
        if count % 3 or count > self.max_input_points:
            raise ValueError("triangle list must contain complete triangles within the input limit")
        if len(marker.colors) not in (0, count):
            raise ValueError("marker color count must be zero or match its point count")
        if (count + self.triangles_per_page * 3 - 1) // (self.triangles_per_page * 3) > self.state.max_pages:
            raise ValueError("marker exceeds map page capacity; increase page size or cache capacity")
        lifetime = float(marker.lifetime.sec) + float(marker.lifetime.nanosec) / 1e9
        if not math.isfinite(now) or not math.isfinite(lifetime) or lifetime < 0:
            raise ValueError("invalid marker lifetime or clock")
        if not count:
            return self._delete(key)
        frame = str(marker.header.frame_id).strip().lstrip("/")
        if not frame:
            raise ValueError("mesh marker has no frame_id")
        tf = None
        if frame != self.state.frame_id.lstrip("/"):
            if resolve_transform is None:
                raise ValueError(f"no transform from {frame} to {self.state.frame_id}")
            tf = point_transform(*resolve_transform(frame, marker.header.stamp))
        translation, rotation = vector(marker.pose.position), quaternion(marker.pose.orientation)
        pose = point_transform(translation, rotation)
        scale = vector(marker.scale)
        global_color = rgba(marker.color)
        pages = []
        step = self.triangles_per_page * 3
        for start in range(0, count, step):
            vertices, colors, indices, lookup = [], [], [], {}
            for index in range(start, min(start + step, count)):
                point = tuple(v*s for v, s in zip(vector(marker.points[index]), scale))
                point = pose(point)
                if tf is not None:
                    point = tf(point)
                if not all(math.isfinite(v) and abs(v) <= FLOAT32_MAX for v in point):
                    raise ValueError("mesh vertex outside finite float32 range")
                color = rgba(marker.colors[index]) if marker.colors else global_color
                vertex_key = (*point, *color)
                vertex_index = lookup.get(vertex_key)
                if vertex_index is None:
                    vertex_index = len(vertices)
                    lookup[vertex_key] = vertex_index
                    vertices.append(point)
                    colors.append(color)
                indices.append(vertex_index)
            pages.append(BlockGeometry(tuple(vertices), tuple(colors), tuple(indices)))
        # Release old tail pages before allocating additional pages elsewhere.
        changes = [((key, part), None) for part in range(len(pages), self._parts.get(key, 0))]
        changes.extend(((key, part), geometry) for part, geometry in enumerate(pages))
        updates = self.state.replace_many(changes)
        self._parts[key] = len(pages)
        if lifetime:
            self._expires[key] = now + lifetime
        else:
            self._expires.pop(key, None)
        return updates

    def expire(self, now):
        updates = []
        for key, deadline in list(self._expires.items()):
            if now >= deadline:
                updates.extend(self._delete(key))
        return updates
