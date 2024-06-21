#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum


class OperationMode(Enum):
    """Global state/mode of the tool."""

    IDLE = 0
    CALIBRATION = 1
    EVALUATION = 2


class ImageViewMode(Enum):
    """Type of image to process and display."""

    SOURCE_UNRECTIFIED = "Source unrectified"
    SOURCE_RECTIFIED = "Source rectified"
    TRAINING_DB_UNRECTIFIED = "Training DB unrectified"
    EVALUATION_DB_UNRECTIFIED = "Evaluation DB unrectified"


class CollectionStatus(Enum):
    """Decision on whether or not to add a sample to the database."""

    NOT_EVALUATED = 1
    REJECTED = 2
    REDUNDANT = 3
    ACCEPTED = 4
