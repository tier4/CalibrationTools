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

#ifndef POINT_CLOUD_ACCUMULATOR__POINT_CLOUD_ACCUMULATOR_HPP_
#define POINT_CLOUD_ACCUMULATOR__POINT_CLOUD_ACCUMULATOR_HPP_

#include <vector>
#include <memory>

#include "message_filters/cache.h"
#include "message_filters/subscriber.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/PCLPointCloud2.h"

namespace point_cloud_accumulator
{
class PointCloudAccumulator : public rclcpp::Node
{
private:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscriber_;
  message_filters::Cache<sensor_msgs::msg::PointCloud2> lidar_cache_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_point_cloud_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  size_t accumulate_frame_num_;

public:
  explicit PointCloudAccumulator(const rclcpp::NodeOptions & node_options);
  void dummyCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> getSensorCacheVector();
  void publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_accumulated,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & msg);
};

}  // namespace point_cloud_accumulator
#endif  // POINT_CLOUD_ACCUMULATOR__POINT_CLOUD_ACCUMULATOR_HPP_
