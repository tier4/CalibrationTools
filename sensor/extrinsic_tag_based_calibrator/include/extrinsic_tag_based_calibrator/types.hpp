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

#ifndef EXTRINSIC_TAG_BASED_CALIBRATOR__TYPES_HPP_
#define EXTRINSIC_TAG_BASED_CALIBRATOR__TYPES_HPP_

enum class EstimationMode {
  NoFiltering,
  SingleStepEstimationKalmanFiltering,
  MultiStepEstimationKalmanFiltering
};

enum class DynamicsModel {
  Static,
  ConstantVelocity,
};

#endif  // EXTRINSIC_TAG_BASED_CALIBRATOR__TYPES_HPP_
