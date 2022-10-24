// Copyright 2022 Tier IV, Inc.
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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_

#include <extrinsic_mapping_based_calibrator/types.hpp>

#include <vector>

class Filter
{
public:
  using Ptr = std::shared_ptr<Filter>;
  using ConstPtr = std::shared_ptr<const Filter>;

  Filter(const CalibrationParameters::Ptr & parameters) : parameters_(parameters), name_("filter")
  {
  }
  virtual ~Filter() {}

  virtual std::vector<CalibrationFrame> filter(
    const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & mapping_data) = 0;
  virtual void setName(const std::string & name) { name_ = name + " filter"; }

protected:
  const CalibrationParameters::Ptr parameters_;
  std::string name_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__FILTER_HPP_
