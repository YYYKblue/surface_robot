// Read the optimized ENU landmark and the reference trajectory's first pose
// from the same Cartographer state used by StartTrajectory.
#include <iomanip>
#include <iostream>
#include <string>

#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/transform/transform.h"

void print_pose(const std::string& name,
                const cartographer::transform::proto::Rigid3d& pose) {
  const auto planar = cartographer::transform::Project2D(
      cartographer::transform::ToRigid3(pose));
  std::cout << name << ' ' << planar.translation().x() << ' '
            << planar.translation().y() << ' ' << planar.rotation().angle() << '\n';
}

int main(int argc, char** argv) {
  if (argc != 4) return 1;
  const auto graph = cartographer::io::DeserializePoseGraphFromFile(argv[1]);
  const int index = std::stoi(argv[2]);
  if (index < 0 || index >= graph.trajectory_size() ||
      graph.trajectory(index).node_size() == 0) return 1;
  std::cout << std::setprecision(17);
  print_pose("reference", graph.trajectory(index).node(0).pose());
  for (const auto& landmark : graph.landmark_poses()) {
    if (landmark.landmark_id().find(argv[3]) == 0)
      print_pose(landmark.landmark_id(), landmark.global_pose());
  }
  return 0;
}
