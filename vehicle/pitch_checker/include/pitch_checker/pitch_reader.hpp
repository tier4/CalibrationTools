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

#ifndef PITCH_CHECKER__PITCH_READER_HPP_
#define PITCH_CHECKER__PITCH_READER_HPP_

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

struct TfInfo
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
};

class PitchReader
{
public:
  explicit PitchReader(const std::string input_file);
  bool getPitch(
    double * pitch, const double x, const double y, const double yaw,
    const double dist_thresh = 10.0, const double yaw_thresh = M_PI_4);
  std::vector<double> comparePitch(const std::string comp_input_file);

private:
  std::vector<TfInfo> tf_infos_;
  bool readCSV(const std::string csv_path, std::vector<TfInfo> * tf_infos);
  std::vector<std::string> split(const std::string & s, char delim);
  bool read_csv_ = false;
};

#endif  // PITCH_CHECKER__PITCH_READER_HPP_
