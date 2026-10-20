#include "Dataset.hpp"
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <iostream>

/// Round-trip a rotated baseline through the actual native YAML exporter.
int main() {
  const auto path = std::filesystem::current_path() / "transform-export-test.yaml";
  try {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.topLeftCorner<3, 3>() =
        (Eigen::AngleAxisd(0.17, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(-0.24, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    matrix.topRightCorner<3, 1>() = Eigen::Vector3d(-0.264, 0.012, -0.007);
    const sm::kinematics::Transformation original(matrix);
    kalibr2::ros::transformStampedToYAML(
        kalibr2::ros::TransformationToROS(original, {"left", "right"}), path);
    const auto transform = YAML::LoadFile(path.string())["transform"];
    const auto q = transform["rotation"];
    const Eigen::Quaterniond exported(q["w"].as<double>(), q["x"].as<double>(),
                                      q["y"].as<double>(), q["z"].as<double>());
    const auto t = transform["translation"];
    const Eigen::Vector3d translation(t["x"].as<double>(), t["y"].as<double>(),
                                      t["z"].as<double>());
    std::filesystem::remove(path);
    if (!exported.toRotationMatrix().isApprox(original.C(), 1e-12) ||
        !translation.isApprox(original.t(), 1e-12)) {
      throw std::runtime_error("Exported quaternion/translation changes the SM transform");
    }
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove(path);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
