#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2023 TIER IV, Inc.
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

# Threshold used for detecting invalid values for expected errors which tends to have significantly large values at the beginning of the bag file
# which is around 1e8 [m]. This value is used to ignore the initial part of the bag file.
THRESHOLD_FOR_INITIALIZED_ERROR = 100.0  # [m]
