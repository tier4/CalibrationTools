// Copyright 2023 Tier IV, Inc.
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

#ifndef EXTRINSIC_GROUND_PLANE_CALIBRATOR__UTILS_HPP_
#define EXTRINSIC_GROUND_PLANE_CALIBRATOR__UTILS_HPP_

#include <vector>

template <class T>
class RingBuffer
{
public:
  void setMaxSize(const int max_size) { max_size_ = max_size; }

  void add(const T & element)
  {
    if (max_size_ == 0) {
      return;
    } else if (current_index_ == data_.size()) {
      data_.push_back(element);
    } else {
      data_[current_index_] = element;
    }

    current_index_ = (current_index_ + 1) % max_size_;
  }

  const std::vector<T> & get() const { return data_; }

  std::vector<T> data_;
  std::size_t current_index_{0};
  int max_size_{0};
};

#endif  // EXTRINSIC_GROUND_PLANE_CALIBRATOR__UTILS_HPP_
