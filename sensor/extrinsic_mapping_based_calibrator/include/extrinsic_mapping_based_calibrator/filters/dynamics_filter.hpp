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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__DYNAMICS_FILTER_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__DYNAMICS_FILTER_HPP_

#include <extrinsic_mapping_based_calibrator/filters/filter.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>

#include <vector>

class DynamicsFilter : public Filter
{
public:
  DynamicsFilter(const LidarCalibrationParameters::Ptr & parameters) : Filter(parameters)
  {
    name_ = "DynamicsFilter";
  }
  DynamicsFilter(const std::string & name, const LidarCalibrationParameters::Ptr & parameters)
  : Filter(parameters)
  {
    setName(name);
  }
  virtual ~DynamicsFilter() {}

  virtual std::vector<CalibrationFrame> filter(
    const std::vector<CalibrationFrame> & calibration_frames,
    MappingData::Ptr & mapping_data) override;
  virtual void setName(const std::string & name) override;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__DYNAMICS_FILTER_HPP_
