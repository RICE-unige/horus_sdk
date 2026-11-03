#include <octomap/ColorOcTree.h>
#include <iostream>

int main()
{
    octomap::ColorOcTree tree(0.2);
    auto voxel = [&](int x, int y, int z, int r, int g, int b) {
        auto* node = tree.updateNode(octomap::point3d((x + 0.5f) * 0.2f,
            (y + 0.5f) * 0.2f, (z + 0.5f) * 0.2f), true);
        node->setColor(r, g, b);
    };
    for (int x = -30; x < 30; ++x)
        for (int y = -20; y < 20; ++y)
        {
            voxel(x, y, 0, 120, 145, 170);
            if (x == -30 || y == 19)
                for (int z = 1; z < 15; ++z)
                    voxel(x, y, z, x == -30 ? 210 : 70, 130, y == 19 ? 210 : 65);
        }
    for (int x = -8; x < 8; ++x)
        for (int y = -5; y < 5; ++y)
            for (int z = 1; z < 7; ++z)
                voxel(x, y, z, 205, 80, 45);
    tree.updateInnerOccupancy();
    tree.writeData(std::cout);
    return std::cout.good() ? 0 : 1;
}
