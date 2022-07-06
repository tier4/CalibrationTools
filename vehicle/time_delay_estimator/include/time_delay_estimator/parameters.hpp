//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef TIME_DELAY_ESTIMATOR__PARAMETERS_HPP_
#define TIME_DELAY_ESTIMATOR__PARAMETERS_HPP_

struct Params
{
  double sampling_hz;
  double estimation_hz;
  double sampling_duration;
  double validation_duration;
  int data_size;
  int total_data_size;
  int validation_size;
  double valid_peak_cross_correlation_threshold;
  double valid_delay_index_ratio;
  double sampling_delta_time;
  double estimation_delta_time;
  double cutoff_hz_input;
  double cutoff_hz_output;
  bool reset_at_disengage;
  bool is_showing_debug_info;
  bool use_interpolation;
  int num_interpolation;
  int estimation_method;
  bool is_test_mode;
};

#endif  // TIME_DELAY_ESTIMATOR__PARAMETERS_HPP_
