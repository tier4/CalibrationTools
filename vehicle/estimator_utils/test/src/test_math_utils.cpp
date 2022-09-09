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

#include "estimator_utils/math_utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

TEST(math_utils, saturation)
{
  using math_utils::saturation;
  double min = 1.0;
  double max = 2.0;
  double val = 2.0;
  const double expect = 2.0;
  EXPECT_DOUBLE_EQ(saturation(val, min, max), expect);
}

TEST(math_utils, normalize)
{
  using math_utils::normalize;
  double min = -1.0;
  double max = 1.0;
  double val = 0.0;
  const double expect = 0.5;
  EXPECT_DOUBLE_EQ(normalize(val, min, max), expect);
}

TEST(math_utils, getLinearInterpolation)
{
  using math_utils::getLinearInterpolation;
  using testing::ElementsAre;
  std::vector<double> input = {0, 3};
  int num_interpolation = 3;
  std::vector<double> output = getLinearInterpolation(input, num_interpolation);
  ASSERT_THAT(output, ElementsAre(0, 1, 2, 3));
}

TEST(math_utils, calcCrossCorrelationCoefficient)
{
  using math_utils::calcCrossCorrelationCoefficient;
  std::vector<double> input = {1, 2, 3, 2, 1, 0};
  std::vector<double> response = {0, 1, 2, 3, 2, 1};
  std::vector<double> weights = {1, 1, 1, 1, 1};
  std::vector<double> output = calcCrossCorrelationCoefficient(input, response, weights, 0.5);
  int delay_index = math_utils::getMaximumIndexFromVector(output);
  EXPECT_EQ(delay_index, 1);
}

TEST(math_utils, Statistics)
{
  using ::testing::ElementsAre;
  math_utils::Statistics stat(1);
  std::vector<double> result = {0, 0.5, 0.81649658092772603};
  std::vector<double> x = {1, 2, 3};
  for (int i = 0; i < 3; ++i) {
    stat.value[0] = x[i];
    math_utils::calcSequentialStddev(stat);
    EXPECT_DOUBLE_EQ(stat.stddev[0], result[i]);
  }
}

TEST(math_utils, getAveragedVector)
{
  using ::testing::ElementsAre;
  std::vector<double> result = {};
  std::vector<double> x = {};
  std::vector<double> empty = math_utils::getAveragedVector(x);
  ASSERT_THAT(empty, ElementsAre());
}

TEST(math_utils, getAveragedVector2)
{
  using ::testing::ElementsAre;
  std::vector<double> result = {-1, 0, 1};
  std::vector<double> x = {1, 2, 3};
  std::vector<double> avg = math_utils::getAveragedVector(x);
  ASSERT_THAT(result, avg);
}

TEST(math_utils, fitToTheSizeOfVector)
{
  std::vector<double> input_stamp = {1, 2, 3, 4, 5, 5, 6, 10};
  std::vector<double> response_stamp = {1, 2, 3, 4, 5, 5, 6, 7};
  std::vector<double> input = {1, 2, 3, 4, 5, 5, 6, 7};
  std::vector<double> response = {1, 2, 3, 4, 5, 5, 6, 7};
  math_utils::fitToTheSizeOfVector(input_stamp, response_stamp, input, response, 5, 1);
  ASSERT_THAT(input, testing::ElementsAre(3, 4, 5, 5, 6));
}
