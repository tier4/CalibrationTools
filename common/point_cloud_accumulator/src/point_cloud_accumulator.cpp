// Copyright 2023 TIER IV, Inc.
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

#include "point_cloud_accumulator/point_cloud_accumulator.hpp"

#include <memory>
#include <vector>

namespace point_cloud_accumulator
{

PointCloudAccumulator::PointCloudAccumulator(const rclcpp::NodeOptions & node_options)
: Node("point_cloud_accumulator", node_options)
{
  using std::placeholders::_1;

  accumulate_frame_num_ = this->declare_parameter("accumulate_frame_num", 10);

  accumulated_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/default", rclcpp::SensorDataQoS());

  auto rmw_qos_profile = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  lidar_subscriber_.subscribe(this, "~/input/default", rmw_qos_profile);
  lidar_cache_.connectInput(lidar_subscriber_);
  lidar_cache_.setCacheSize(accumulate_frame_num_);
  lidar_cache_.registerCallback(std::bind(&PointCloudAccumulator::dummyCallback, this, _1));
}

void PointCloudAccumulator::dummyCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {
    RCLCPP_ERROR(this->get_logger(), "Sensor topic is none");
  } else {
    auto lidar_cache_vector = getSensorCacheVector();
    if (lidar_cache_vector.size() < accumulate_frame_num_) {
      RCLCPP_DEBUG(
        this->get_logger(), "cant get sensor cache. vector size = %ld", lidar_cache_vector.size());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "get sensor topic. size = %ld", msg->data.size());
      // do publish
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_sensor(new pcl::PointCloud<pcl::PointXYZ>);
      for (auto & cache : lidar_cache_vector) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cache, *temp_cloud);
        *pcl_sensor += *temp_cloud;
      }
      publishPointCloud(pcl_sensor, lidar_cache_vector.front());
    }
  }
}

std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>
PointCloudAccumulator::getSensorCacheVector()
{
  rclcpp::Time start_time = lidar_cache_.getOldestTime();
  rclcpp::Time end_time = lidar_cache_.getLatestTime();
  return lidar_cache_.getInterval(start_time, end_time);
}

void PointCloudAccumulator::publishPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_accumulated,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & msg)
{
  sensor_msgs::msg::PointCloud2 accumulated_point_cloud_msg;
  pcl::toROSMsg(*pcl_accumulated, accumulated_point_cloud_msg);
  accumulated_point_cloud_msg.header.frame_id = "base_link";
  accumulated_point_cloud_msg.header.stamp.nanosec = msg->header.stamp.nanosec;
  accumulated_point_cloud_msg.header.stamp.sec = msg->header.stamp.sec;
  accumulated_point_cloud_pub_->publish(accumulated_point_cloud_msg);
}

}  // namespace point_cloud_accumulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<point_cloud_accumulator::PointCloudAccumulator>(node_options);

  rclcpp::spin(node);
}
