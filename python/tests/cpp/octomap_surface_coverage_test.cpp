#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <octomap/OcTree.h>
#include "octomap_surface_coverage.hpp"

int main()
{
  unsigned cases = 0;
  for (int shift : {-8, 0})
  {
    for (int axis = 0; axis < 3; ++axis)
    {
      for (int side : {-1, 1})
      {
        for (unsigned mask = 0; mask < 16; ++mask)
        {
          octomap::OcTree tree(1.0);
          for (int x = 0; x < 2; ++x)
            for (int y = 0; y < 2; ++y)
              for (int z = 0; z < 2; ++z)
                tree.setNodeValue(shift + x + 0.5, shift + y + 0.5, shift + z + 0.5, 1.0f);
          int covered = 0;
          const int u = axis == 0 ? 1 : 0;
          const int v = axis == 2 ? 1 : 2;
          for (int i = 0; i < 4; ++i)
          {
            if (!(mask & (1u << i))) continue;
            std::array<double, 3> p;
            p[axis] = shift + (side > 0 ? 2.5 : -0.5);
            p[u] = shift + (i % 2) + 0.5;
            p[v] = shift + (i / 2) + 0.5;
            tree.setNodeValue(p[0], p[1], p[2], 1.0f);
            ++covered;
          }
          tree.updateInnerOccupancy();
          tree.prune();
          auto * source = tree.search(shift + 1.0, shift + 1.0, shift + 1.0, tree.getTreeDepth() - 1);
          assert(source && !tree.nodeHasChildren(source));
          std::array<double, 3> face{shift + 1.0, shift + 1.0, shift + 1.0};
          face[axis] = shift + (side > 0 ? 2.0 : 0.0);
          double area = 0;
          horus::exposed_face_patches(tree, face, 2.0, tree.getTreeDepth() - 1, axis, side,
            [&](const std::array<double, 3> & p, double size) {
              assert(p[axis] == face[axis]);
              assert(size == 1.0 || size == 2.0);
              area += size * size;
            });
          assert(std::abs(area - (4 - covered)) < 1e-9);
          ++cases;
        }
      }
    }
  }
  std::cout << "PASS " << cases << " mixed-resolution face cases (all axes/sides, negative coordinates)\n";
}
