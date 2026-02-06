#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include <opencv2/opencv.hpp>
#include <string>

namespace drake {
namespace systems {

/**
 * Displays Drake camera images in an OpenCV window.
 * Updates at configurable frame rate (default 30 FPS).
 */
class CameraViewer : public LeafSystem<double> {
 public:
  /**
   * Constructor.
   * @param window_name Name of the OpenCV window
   * @param fps Frame rate for display updates (default: 30)
   */
  explicit CameraViewer(const std::string& window_name = "FPV Camera",
                        double fps = 30.0);
  
  ~CameraViewer() override;

 private:
  // Periodic event handler to display images
  void DisplayImage(const Context<double>& context) const;
  
  std::string window_name_;
  mutable bool window_created_{false};
};

}  // namespace systems
}  // namespace drake