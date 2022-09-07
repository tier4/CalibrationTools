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

#ifndef VEHICLE_CMD_ANALYZER__DEBUG_VALUES_HPP_
#define VEHICLE_CMD_ANALYZER__DEBUG_VALUES_HPP_

#include <array>

/// Debug Values used for debugging or controller tuning
class DebugValues
{
public:
  /// Types of debug values
  enum class TYPE {
    DT = 0,
    CURRENT_TARGET_VEL = 1,
    CURRENT_TARGET_D_VEL = 2,
    CURRENT_TARGET_DD_VEL = 3,
    CURRENT_TARGET_ACC = 4,
    CURRENT_TARGET_D_ACC = 5,
    CURRENT_TARGET_LATERAL_ACC = 6,
    SIZE  // this is the number of enum elements
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getValuesIdx(const TYPE & type) const { return static_cast<int>(type); }
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE & type, const double val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const double val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

#endif  // VEHICLE_CMD_ANALYZER__DEBUG_VALUES_HPP_
