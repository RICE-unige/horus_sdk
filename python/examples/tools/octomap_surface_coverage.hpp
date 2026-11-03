#pragma once

#include <array>

namespace horus
{
// Visit only uncovered patches of a leaf face. A neighbor at the same depth
// may be an inner node: its occupancy is not proof that it covers the face.
template <typename Tree, typename Emit>
void exposed_face_patches(
  const Tree & tree, const std::array<double, 3> & face_center,
  double size, unsigned depth, int axis, int side, Emit && emit)
{
  auto probe = face_center;
  probe[axis] += side * tree.getResolution() * 0.25;
  const auto * neighbor = tree.search(probe[0], probe[1], probe[2], depth);
  if (neighbor == nullptr || !tree.nodeHasChildren(neighbor))
  {
    if (neighbor == nullptr || !tree.isNodeOccupied(neighbor)) emit(face_center, size);
    return;
  }
  const int u = axis == 0 ? 1 : 0;
  const int v = axis == 2 ? 1 : 2;
  for (int du : {-1, 1})
  {
    for (int dv : {-1, 1})
    {
      auto child = face_center;
      child[u] += du * size * 0.25;
      child[v] += dv * size * 0.25;
      exposed_face_patches(tree, child, size * 0.5, depth + 1, axis, side, emit);
    }
  }
}
}  // namespace horus
