#pragma once
#include <kalibr2/Image.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Preserve the existing CLI's algorithm calls with plain data and YAML adapters.
namespace kalibr2::ros {
struct CameraConfig {
  std::string camera_name;
  std::unique_ptr<kalibr2::ImageReader> reader;
  std::string model;
  std::optional<double> focal_length;
};
struct CalibrationConfig {
  std::vector<CameraConfig> cameras;
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
};
/// Read the offline dataset configuration without ROS messages or middleware.
CalibrationConfig ConfigFromYaml(const std::string&, const std::optional<std::string>&);
/// Write CameraInfo-compatible plain YAML from the estimated geometry.
void CalibratorToYAML(const boost::shared_ptr<CameraCalibratorBase>&, const std::string&,
  const std::string&, size_t, size_t, const std::filesystem::path&);
struct PlainTransform {
  sm::kinematics::Transformation value;
  std::pair<std::string, std::string> frames;
};
/// Retain an estimated transform and its camera names as plain C++ data.
PlainTransform TransformationToROS(const sm::kinematics::Transformation&,
  std::pair<std::string, std::string>);
/// Write a TransformStamped-compatible YAML mapping without ROS type support.
void transformStampedToYAML(const PlainTransform&, const std::filesystem::path&);
/// Reject unsupported multi-camera output in the deliberately stereo-only build.
std::vector<PlainTransform> TransformationsToTFMessage(
  const std::vector<sm::kinematics::Transformation>&,
  const std::vector<std::pair<std::string, std::string>>&);
/// Reject unsupported multi-camera output in the deliberately stereo-only build.
void tfMessageToYAML(const std::vector<PlainTransform>&, const std::filesystem::path&);
}
