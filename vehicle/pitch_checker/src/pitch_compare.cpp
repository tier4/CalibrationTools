//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#include "pitch_checker/pitch_compare.hpp"

#include "pitch_checker/pitch_reader.hpp"

#include <string>

namespace pitch_compare
{
PitchCompareNode::PitchCompareNode(const rclcpp::NodeOptions & node_options)
: Node("pitch_compare", node_options)
{
  const auto base_pitch_csv =
    this->declare_parameter<std::string>("base_pitch_csv", "base_pitch.csv");
  const auto pitch_csv = this->declare_parameter<std::string>("pitch_csv", "pitch.csv");

  PitchReader node(base_pitch_csv);
  const auto dif_pitch_vec = node.comparePitch(pitch_csv);

  for (const auto dif_pitch : dif_pitch_vec) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pitch_compare"), dif_pitch);
  }
}
}  // namespace pitch_compare
