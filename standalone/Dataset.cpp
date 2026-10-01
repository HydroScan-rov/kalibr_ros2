#include "Dataset.hpp"
#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>
#include <boost/make_shared.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace kalibr2::ros {
/// Lazily read lossless grayscale frames from a timestamp-indexed offline dataset.
class DatasetReader final : public kalibr2::ImageReader {
 public:
  /// Load only the compact frame index, preserving integer sensor timestamps.
  explicit DatasetReader(const std::filesystem::path& index) : root_(index.parent_path()) {
    std::ifstream input(index);
    if (!input) throw std::runtime_error("Cannot open dataset index: " + index.string());
    std::string line;
    while (std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty() || line[0] == '#') continue;
      auto split = line.find(',');
      if (split == std::string::npos) throw std::runtime_error("Invalid frame index");
      int64_t stamp = std::stoll(line.substr(0, split));
      if (!frames_.empty() && stamp <= frames_.back().first)
        throw std::runtime_error("Non-monotonic image timestamps");
      frames_.emplace_back(stamp, line.substr(split + 1));
    }
    if (frames_.empty()) throw std::runtime_error("Dataset contains no images");
  }
  /// Decode one frame, retaining its sensor timestamp and native geometry.
  kalibr2::Image ReadNext() override {
    if (!HasNext()) throw std::runtime_error("Dataset exhausted");
    const auto& [stamp, name] = frames_.at(position_++);
    cv::Mat image = cv::imread((root_ / name).string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) throw std::runtime_error("Cannot decode dataset image: " + name);
    image_width_ = image.cols; image_height_ = image.rows;
    return {aslam::Time(stamp / 1000000000LL, stamp % 1000000000LL), image};
  }
  /// Report whether an unread indexed frame remains.
  bool HasNext() const override { return position_ < frames_.size(); }
  /// Return the number of indexed frames without loading their pixels.
  size_t MessageCount() override { return frames_.size(); }
 private:
  std::filesystem::path root_;
  std::vector<std::pair<int64_t, std::string>> frames_;
  size_t position_ = 0;
};

/// Write one plain YAML document and detect disk-full and other write failures.
static void save(const YAML::Node& node, const std::filesystem::path& path) {
  std::ofstream output(path);
  output << node << '\n'; output.flush();
  if (!output) throw std::runtime_error("Cannot write calibration output: " + path.string());
}

CalibrationConfig ConfigFromYaml(const std::string& file, const std::optional<std::string>&) {
  auto yaml = YAML::LoadFile(file); const auto& board = yaml["board"];
  if (board["target_type"].as<std::string>() != "aprilgrid")
    throw std::runtime_error("Only AprilGrid is supported");
  auto options = aslam::cameras::GridCalibrationTargetAprilgridv2::AprilgridOptionsv2();
  options.detectorParameters->markerBorderBits = 2;
  CalibrationConfig result;
  result.target = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgridv2>(
    board["tagRows"].as<int>(), board["tagCols"].as<int>(), board["tagSize"].as<double>(),
    board["tagSpacing"].as<double>(), options);
  for (const auto& entry : yaml["cameras"]) {
    auto config = entry.second; std::string model = config["model"].as<std::string>();
    if (model != "pinhole-radtan" && model != "pinhole-equi")
      throw std::runtime_error("Unsupported camera model: " + model);
    result.cameras.push_back({entry.first.as<std::string>(),
      std::make_unique<DatasetReader>(config["source"]["index"].as<std::string>()), model,
      config["focal_length_fallback"].as<double>()});
  }
  if (result.cameras.size() != 2) throw std::runtime_error("Exactly two cameras are required");
  return result;
}

void CalibratorToYAML(const boost::shared_ptr<CameraCalibratorBase>& calibrator,
    const std::string& model, const std::string&, size_t width, size_t height,
    const std::filesystem::path& path) {
  Eigen::MatrixXd k, d;
  calibrator->camera_geometry()->getParameters(k, true, false, false);
  calibrator->camera_geometry()->getParameters(d, false, true, false);
  YAML::Node out; out["width"] = width; out["height"] = height;
  out["distortion_model"] = model == "pinhole-radtan" ? "plumb_bob" : "equidistant";
  for (int i = 0; i < d.size(); ++i) out["d"].push_back(d(i));
  for (double v : {k(0), 0.0, k(2), 0.0, k(1), k(3), 0.0, 0.0, 1.0}) out["k"].push_back(v);
  save(out, path);
}

PlainTransform TransformationToROS(const sm::kinematics::Transformation& value,
    std::pair<std::string, std::string> frames) { return {value, std::move(frames)}; }

void transformStampedToYAML(const PlainTransform& tf, const std::filesystem::path& path) {
  YAML::Node out; const auto& t = tf.value.t(); const auto& q = tf.value.q();
  auto translation = out["transform"]["translation"];
  translation["x"] = t.x(); translation["y"] = t.y(); translation["z"] = t.z();
  auto rotation = out["transform"]["rotation"];
  rotation["x"] = q.x(); rotation["y"] = q.y(); rotation["z"] = q.z(); rotation["w"] = q.w();
  save(out, path);
}

std::vector<PlainTransform> TransformationsToTFMessage(
    const std::vector<sm::kinematics::Transformation>&,
    const std::vector<std::pair<std::string, std::string>>&) {
  throw std::runtime_error("Only stereo calibration is supported");
}
void tfMessageToYAML(const std::vector<PlainTransform>&, const std::filesystem::path&) {
  throw std::runtime_error("Only stereo calibration is supported");
}
}
