#include <lems/data/dataset.hpp>

#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

void print_camera(const char* label, const lems::data::Camera& camera) {
  std::cout << "  " << label << ": "
            << camera.resolution.at(0) << "x" << camera.resolution.at(1)
            << ", K =\n" << camera.K << "\n";
  std::cout << "    intrinsics:";
  for (double value : camera.intrinsics) std::cout << " " << value;
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: lems-data-inspect CONFIG.yaml\n";
    return 2;
  }

  try {
    auto dataset = lems::data::open_dataset(
        lems::data::load_config(argv[1]));
    const auto& files = dataset->file_info();
    const auto& cameras = dataset->camera_info();

    std::cout << "dataset: " << files.dataset_type << "\n"
              << "root: " << files.dataset_path << "\n"
              << "sequence: " << files.sequence_name << "\n"
              << "frames: " << dataset->size() << " (" << dataset->width()
              << "x" << dataset->height() << ")\n"
              << "pose/reference ground truth: "
              << (dataset->has_ground_truth() ? "yes" : "no") << "\n";
    print_camera("left", cameras.left);
    print_camera("right", cameras.right);
    if (dataset->stereo_calibration()) {
      const auto& stereo = *dataset->stereo_calibration();
      std::cout << "stereo baseline: " << stereo.baseline << " m\n"
                << "stereo R (target <- reference):\n"
                << stereo.R_target_reference << "\n"
                << "stereo t (target <- reference): "
                << stereo.t_target_reference.transpose() << "\n";
    }

    auto iterator = dataset->iterate();
    std::size_t sets = 0;
    std::size_t poses = 0;
    std::size_t disparities = 0;
    std::size_t masks = 0;
    while (auto set = iterator->next()) {
      if (sets < 3) {
        std::cout << "FrameSet " << set->index << " @ " << set->timestamp_ns
                  << " ns: " << set->frames.size() << " camera observations\n";
        for (const auto& frame : set->frames)
          std::cout << "  " << frame.camera << " @ " << frame.timestamp_ns
                    << " ns: " << frame.image_path << "\n";
      }
      for (const auto& frame : set->frames) {
        poses += frame.ground_truth.has_value();
        disparities += frame.metadata.disparity_path.has_value();
        masks += frame.metadata.occlusion_mask_path.has_value();
      }
      ++sets;
    }
    std::cout << "FrameSets: " << sets << "\n"
              << "frames with poses: " << poses << "\n"
              << "disparity references: " << disparities << "\n"
              << "occlusion-mask references: " << masks << "\n";
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
