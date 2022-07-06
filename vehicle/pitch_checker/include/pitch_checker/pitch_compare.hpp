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

#ifndef PITCH_CHECKER__PITCH_COMPARE_HPP_
#define PITCH_CHECKER__PITCH_COMPARE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace pitch_compare
{
class PitchCompareNode : public rclcpp::Node
{
public:
  explicit PitchCompareNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace pitch_compare

#endif  // PITCH_CHECKER__PITCH_COMPARE_HPP_
