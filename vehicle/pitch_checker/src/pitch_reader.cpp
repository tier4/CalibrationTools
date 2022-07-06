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

#include <string>
#include <limits>
#include <vector>
#include "pitch_checker/pitch_reader.hpp"

PitchReader::PitchReader(const std::string input_file)
{
  read_csv_ = (readCSV(input_file, &tf_infos_));
}

bool PitchReader::getPitch(
  double * pitch, const double x, const double y, const double yaw, const double dist_thresh,
  const double yaw_thresh)
{
  if (!read_csv_) {return false;}

  double min_dist = std::numeric_limits<double>::max();
  bool success_search = false;
  for (const auto & tf_info : tf_infos_) {
    const double dist = std::hypot(x - tf_info.x, y - tf_info.y);
    if (dist < dist_thresh && dist < min_dist && std::fabs(yaw - tf_info.yaw) < yaw_thresh) {
      min_dist = dist;
      *pitch = tf_info.pitch;
      success_search = true;
    }
  }
  return success_search;
}

std::vector<double> PitchReader::comparePitch(const std::string comp_input_file)
{
  std::vector<double> pitches;
  std::vector<TfInfo> comp_tf_infos;
  if (!readCSV(comp_input_file, &comp_tf_infos)) {
    return pitches;
  }

  for (const auto & tf_info : comp_tf_infos) {
    double pitch;
    if (getPitch(&pitch, tf_info.x, tf_info.y, tf_info.yaw)) {
      pitches.emplace_back(pitch - tf_info.pitch);
    }
  }
  return pitches;
}

bool PitchReader::readCSV(const std::string csv_path, std::vector<TfInfo> * tf_infos)
{
  std::vector<std::vector<std::string>> result;
  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    std::vector<std::string> strvec = split(line, ',');
    result.emplace_back(strvec);
  }

  for (size_t i = 1; i < result.size(); i++) {
    TfInfo tf_info;
    tf_info.x = std::stod(result.at(i).at(0));
    tf_info.y = std::stod(result.at(i).at(1));
    tf_info.z = std::stod(result.at(i).at(2));
    tf_info.yaw = std::stod(result.at(i).at(3));
    tf_info.pitch = std::stod(result.at(i).at(4));
    tf_infos->emplace_back(tf_info);
  }

  return true;
}

std::vector<std::string> PitchReader::split(const std::string & original, char delim)
{
  std::vector<std::string> elems;
  std::stringstream sstr(original);
  std::string elem;
  while (getline(sstr, elem, delim)) {
    if (!elem.empty()) {elems.push_back(elem);}
  }
  return elems;
}
