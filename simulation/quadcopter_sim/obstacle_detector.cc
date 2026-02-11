#include "obstacle_detector.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace drake {
namespace systems {

ObstacleDetector::ObstacleDetector() {
  this->DeclareAbstractInputPort("depth_image", Value<sensors::ImageDepth32F>());
  this->DeclareVectorInputPort("desired_velocity", 3);
  this->DeclareVectorOutputPort("safe_velocity", 3, &ObstacleDetector::ComputeSafeVelocity);
  
  this->DeclarePeriodicPublishEvent(1.0, 0.0, &ObstacleDetector::CheckObstacles);
  
  std::cout << "ObstacleDetector created (safety distance: " 
            << safety_distance_ << "m)" << std::endl;
}

Eigen::Vector4d ObstacleDetector::ProcessControl(
    const sensors::ImageDepth32F& depth_image,
    const Eigen::Vector4d& desired_control) const {
  
  // Convert depth to OpenCV
  cv::Mat depth_mat(depth_image.height(), depth_image.width(), CV_32FC1,
                    (void*)depth_image.at(0, 0));
  
  // Check center region
  int h = depth_mat.rows;
  int w = depth_mat.cols;
  cv::Rect center_roi(w/3, h/3, w/3, h/3);
  cv::Mat center_depth = depth_mat(center_roi);
  
  // Find minimum valid depth
  cv::Mat valid_mask = (center_depth > 0.05) & (center_depth < 10.0);
  double min_depth = 100.0;
  
  if (cv::countNonZero(valid_mask) > 5) {
    cv::minMaxLoc(center_depth, &min_depth, nullptr, nullptr, nullptr, valid_mask);
  }
  
  // Copy input to output
  Eigen::Vector4d safe_control = desired_control;
  
  // BLOCK FORWARD MOTION if obstacle too close
  if (min_depth < safety_distance_) {
    // Your control format: [thrust, roll, pitch, yaw]
    // Forward motion = positive PITCH (control_input(2))
    
    if (safe_control(2) > 0.0) {  // Trying to pitch forward
      static int block_counter = 0;
      if (block_counter % 10 == 0) {
        std::cout << "🛑 BLOCKING forward pitch! Obstacle at " 
                  << min_depth << "m" << std::endl;
      }
      block_counter++;
      
      safe_control(2) = 0.0;  // Zero out forward pitch
    }
  }
  
  return safe_control;
}

void ObstacleDetector::CheckObstacles(const Context<double>& context) const {
  const auto& depth_image = this->get_input_port(0).Eval<sensors::ImageDepth32F>(context);
  
  cv::Mat depth_mat(depth_image.height(), depth_image.width(), CV_32FC1,
                    (void*)depth_image.at(0, 0));
  
  int h = depth_mat.rows;
  int w = depth_mat.cols;
  cv::Rect center_roi(w/3, h/3, w/3, h/3);
  cv::Mat center_depth = depth_mat(center_roi);
  
  cv::Mat valid_mask = (center_depth > 0.05) & (center_depth < 10.0);
  double min_depth = 100.0;
  int valid_count = cv::countNonZero(valid_mask);
  
  if (valid_count > 5) {
    cv::minMaxLoc(center_depth, &min_depth, nullptr, nullptr, nullptr, valid_mask);
  }
  
  std::cout << "🔍 Obstacle Check: min_depth=" << min_depth 
            << "m, valid_pixels=" << valid_count 
            << ", threshold=" << safety_distance_ << "m";
  
  if (min_depth < safety_distance_) {
    std::cout << " ⚠️  OBSTACLE!" << std::endl;
  } else {
    std::cout << " ✅ Clear" << std::endl;
  }
}

void ObstacleDetector::ComputeSafeVelocity(const Context<double>& context,
                                           BasicVector<double>* output) const {
  output->SetFromVector(Eigen::Vector3d::Zero());
}

}  // namespace systems
}  // namespace drake