// Copyright 2024 TIER IV, Inc.
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

#ifndef MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_
#define MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_

#include <mapping_based_calibrator/types.hpp>

#include <memory>
#include <string>
#include <vector>

class Filter
{
public:
  enum class FilterType {
    CameraFilter,
    LidarFilter,
  };

  using Ptr = std::shared_ptr<Filter>;
  using ConstPtr = std::shared_ptr<const Filter>;

  Filter(const FilterType & filter_type, const CalibrationParameters::Ptr & parameters)
  : filter_type_(filter_type), parameters_(parameters), name_("filter")
  {
  }
  virtual ~Filter() {}

  virtual std::vector<CalibrationFrame> filter(
    const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & mapping_data) = 0;
  virtual void setName(const std::string & name) { name_ = name + " filter"; }

protected:
  FilterType filter_type_;
  const CalibrationParameters::Ptr parameters_;
  std::string name_;
};

#endif  // MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_
