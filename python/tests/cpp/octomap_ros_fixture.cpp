#include <octomap/ColorOcTree.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
  const bool occupied = argc < 2 || std::string(argv[1]) != "empty";
  octomap::ColorOcTree tree(0.1);
  auto *left = tree.updateNode(octomap::point3d(0.05f, 0.05f, 0.05f), occupied);
  auto *right = tree.updateNode(octomap::point3d(0.15f, 0.05f, 0.05f), occupied);
  left->setColor(255, 0, 0);
  right->setColor(0, 255, 0);
  tree.updateInnerOccupancy();
  std::ostringstream bytes(std::ios::out | std::ios::binary);
  tree.writeData(bytes);
  for (const unsigned char value : bytes.str())
    std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(value);
  std::cout << '\n';
}
