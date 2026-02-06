#include "camera_viewer.h"
#include <iostream>

namespace drake {
namespace systems {

CameraViewer::CameraViewer(const std::string& window_name, double fps)
    : window_name_(window_name) {
  
  // Declare input port for RGBA images
  this->DeclareAbstractInputPort(
      "image",
      drake::Value<sensors::ImageRgba8U>());
  
  // Periodic publish for display
  const double period = 1.0 / fps;
  this->DeclarePeriodicPublishEvent(
      period, 
      0.0,
      &CameraViewer::DisplayImage);
  
  std::cout << "CameraViewer created: '" << window_name_ 
            << "' @ " << fps << " FPS" << std::endl;
}

CameraViewer::~CameraViewer() {
  if (window_created_) {
    cv::destroyWindow(window_name_);
  }
}

void CameraViewer::DisplayImage(const Context<double>& context) const {
  // Get image from input port
  const auto& drake_image = this->get_input_port(0)
      .Eval<sensors::ImageRgba8U>(context);
  
  // Create window on first call
  if (!window_created_) {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, drake_image.width(), drake_image.height());
    window_created_ = true;
  }
  
  // Convert Drake RGBA to OpenCV Mat
  cv::Mat rgba_image(
      drake_image.height(),
      drake_image.width(),
      CV_8UC4,
      (void*)drake_image.at(0, 0)
  );
  
   // Convert RGBA to BGR for display
  cv::Mat bgr_image;
  cv::cvtColor(rgba_image, bgr_image, cv::COLOR_RGBA2BGR);
  
  // ========================================================================
  // ROTATE 90° COUNTER-CLOCKWISE to fix orientation
  // ========================================================================
  cv::Mat display_image;
  cv::rotate(bgr_image, display_image, cv::ROTATE_90_COUNTERCLOCKWISE);
  
  // Add minimal overlay (just time, in top-left corner, small)
  const double sim_time = context.get_time();
  char time_text[32];
  snprintf(time_text, sizeof(time_text), "%.2fs", sim_time);
  
  cv::putText(
      display_image,
      time_text,
      cv::Point(10, 25),              // Top-left
      cv::FONT_HERSHEY_SIMPLEX,
      0.3,                            // Smaller font
      cv::Scalar(0, 255, 0),          // Green
      1,                              // Thinner
      cv::LINE_AA
  );
  
  // Add simple crosshair (optional)
  int cx = display_image.cols / 2;
  int cy = display_image.rows / 2;
  
  // Small crosshair
  cv::line(display_image, 
           cv::Point(cx - 15, cy), 
           cv::Point(cx + 15, cy),
           cv::Scalar(255, 255, 255), 1);
  cv::line(display_image, 
           cv::Point(cx, cy - 15), 
           cv::Point(cx, cy + 15),
           cv::Scalar(255, 255, 255), 1);
  cv::circle(display_image, cv::Point(cx, cy), 3, cv::Scalar(255, 255, 255), 1);
  
  // Display
  cv::imshow(window_name_, display_image);
  cv::waitKey(1);
}

}  // namespace systems
}  // namespace drake