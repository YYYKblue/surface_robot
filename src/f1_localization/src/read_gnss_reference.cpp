// Recover the selected trajectory's reference node and geographic registration.
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/transform/transform.h"

void print_pose(const std::string& name, const cartographer::transform::proto::Rigid3d& pose) {
  const auto xy = cartographer::transform::Project2D(cartographer::transform::ToRigid3(pose));
  std::cout << name << ' ' << xy.translation().x() << ' '
            << xy.translation().y() << ' ' << xy.rotation().angle() << '\n';
}

int main(int argc, char** argv) {
  if ((argc != 3 && argc != 4) || !std::ifstream(argv[1])) return 1;
  cartographer::io::ProtoStreamReader reader(argv[1]);
  cartographer::io::ProtoStreamDeserializer stream(&reader);
  const auto& graph = stream.pose_graph();
  const int id = std::stoi(argv[2]);
  const auto& trajectories = graph.trajectory();
  auto trajectory = std::find_if(trajectories.begin(), trajectories.end(),
      [id](const auto& t) { return t.trajectory_id() == id; });
  if (trajectory == trajectories.end() || !trajectory->node_size()) return 1;
  std::cout << std::setprecision(17);
  print_pose("reference", trajectory->node(0).pose());
  const cartographer::mapping::proto::PoseGraph::LandmarkPose* origin = nullptr;
  for (const auto& marker : graph.landmark_poses()) {
    const auto& name = marker.landmark_id();
    if (argc == 4 && name.rfind(argv[3], 0) != 0) continue;
    if (name.rfind("f1_enu_v1:", 0) != 0 && name.rfind("f1_gga_v2:", 0) != 0) continue;
    // Several distinct datums cannot identify a unique geographic registration.
    if (origin != nullptr) return 0;
    origin = &marker;
  }
  if (origin == nullptr) return 0;
  if (origin->landmark_id().rfind("f1_enu_v1:", 0) == 0) {
    print_pose(origin->landmark_id(), origin->global_pose());
    return 0;
  }
  cartographer::mapping::proto::SerializedData data;
  while (stream.ReadNextSerializedData(&data)) {
    if (data.has_trajectory_data() && data.trajectory_data().trajectory_id() == id &&
        data.trajectory_data().has_fixed_frame_origin_in_map()) {
      print_pose(origin->landmark_id(), data.trajectory_data().fixed_frame_origin_in_map());
      return 0;
    }
  }
  return reader.eof() ? 0 : 1;
}
