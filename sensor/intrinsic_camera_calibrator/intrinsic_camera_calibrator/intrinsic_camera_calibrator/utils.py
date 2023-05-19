#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
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

import cv2
from intrinsic_camera_calibrator.camera_model import CameraModel
import numpy as np
import ruamel.yaml
import yaml


def to_grayscale(img: np.array) -> np.array:
    """Convert a image to grayscale."""
    if len(img.shape) == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        return img


def save_intrinsics(camera_model: CameraModel, alpha, camera_name, file_path: str):
    data = camera_model.as_dict(alpha)
    data["camera_name"] = camera_name

    def format_list(data):
        if isinstance(data, list):
            retval = ruamel.yaml.comments.CommentedSeq(data)
            retval.fa.set_flow_style()
            return retval
        elif isinstance(data, dict):
            return {k: format_list(v) for k, v in data.items()}
        else:
            return data

    data = format_list(data)

    with open(file_path, "w") as f:
        yaml = ruamel.yaml.YAML()
        yaml.dump(data, f)


def load_intrinsics(file_path: str):
    with open(file_path, "r") as stream:
        data = yaml.safe_load(stream)

    camera_model = CameraModel()
    camera_model.from_dict(data)

    return camera_model
