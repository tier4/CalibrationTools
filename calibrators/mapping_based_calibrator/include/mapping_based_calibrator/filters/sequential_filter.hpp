// Copyright 2024 Tier IV, Inc.
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

#ifndef MAPPING_BASED_CALIBRATOR__FILTERS__SEQUENTIAL_FILTER_HPP_
#define MAPPING_BASED_CALIBRATOR__FILTERS__SEQUENTIAL_FILTER_HPP_

#include <mapping_based_calibrator/filters/filter.hpp>
#include <mapping_based_calibrator/types.hpp>

#include <memory>
#include <string>
#include <vector>

class SequentialFilter : public Filter
{
public:
  SequentialFilter(
    const Filter::FilterType & filter_type, const CalibrationParameters::Ptr & parameters)
  : Filter(filter_type, parameters)
  {
  }
  SequentialFilter(
    const Filter::FilterType & filter_type, const std::string & name,
    const CalibrationParameters::Ptr & parameters)
  : Filter(filter_type, parameters)
  {
    setName(name);
  }

  SequentialFilter(
    const Filter::FilterType & filter_type, const std::string & name,
    const CalibrationParameters::Ptr & parameters, const std::vector<Filter::Ptr> & filters)
  : Filter(filter_type, parameters), filters_(filters)
  {
    setName(name);
  }

  SequentialFilter(
    const Filter::FilterType & filter_type, const CalibrationParameters::Ptr & parameters,
    const std::vector<Filter::Ptr> & filters)
  : Filter(filter_type, parameters), filters_(filters)
  {
  }
  virtual ~SequentialFilter() {}

  std::vector<CalibrationFrame> filter(
    const std::vector<CalibrationFrame> & calibration_frames,
    MappingData::Ptr & mapping_data) override
  {
    std::vector<CalibrationFrame> out = calibration_frames;

    for (auto & filter : filters_) {
      out = filter->filter(out, mapping_data);
    }

    return out;
  }

  void addFilter(Filter::Ptr & filter) { filters_.push_back(filter); }

private:
  std::vector<Filter::Ptr> filters_;
};

#endif  // MAPPING_BASED_CALIBRATOR__FILTERS__SEQUENTIAL_FILTER_HPP_
