// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXTRINSIC_DUMMY_CALIBRATOR__EXTRINSIC_DUMMY_CALIBRATOR_HPP_
#define EXTRINSIC_DUMMY_CALIBRATOR__EXTRINSIC_DUMMY_CALIBRATOR_HPP_

#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/clock.hpp"

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/registration/gicp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tier4_calibration_msgs/srv/extrinsic_calibrator.hpp"


namespace extrinsic_dummy_calibrator
{

class ExtrinsicDummyCalibrator : public rclcpp::Node
{
private:
  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr server_;

  std::mutex mutex_;
  std::string parent_frame_;
  std::string child_frame_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;


public:
  explicit ExtrinsicDummyCalibrator(const rclcpp::NodeOptions & node_options);
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);
};

}  // namespace extrinsic_dummy_calibrator
#endif  // EXTRINSIC_DUMMY_CALIBRATOR__EXTRINSIC_DUMMY_CALIBRATOR_HPP_
