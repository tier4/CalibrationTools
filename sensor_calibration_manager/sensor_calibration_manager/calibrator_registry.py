#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

from collections import defaultdict
import logging
from typing import Callable
from typing import List

from sensor_calibration_manager.calibrator_base import CalibratorBase


class CalibratorRegistry:
    """The factory class to register and execute calibrators."""

    logger = logging.getLogger(__name__)
    registry = defaultdict(lambda: defaultdict(CalibratorBase))
    """ Internal registry for available calibrators """

    @classmethod
    def getProjects(cls) -> List:
        return list(cls.registry.keys())

    @classmethod
    def getProjectCalibrators(cls, project_name) -> List:
        return list(cls.registry[project_name].keys())

    @classmethod
    def register_calibrator(cls, project_name: str, calibrator_name: str) -> Callable:
        """
        Class method to register implementations of the CalibratorBase class into the internal registry.

        Args:
            project_name (str): The name of the calibration project.
            calibrator_name (str): The name of the calibrator.

        Returns
        -------
            wrapper

        """

        def inner_wrapper(wrapped_class: CalibratorBase) -> CalibratorBase:
            cls.logger.info(f"Adding {wrapped_class.__name__}")
            if project_name in cls.registry and calibrator_name in cls.registry[project_name]:
                cls.logger.warning(
                    f"Calibrator project={project_name} name={calibrator_name} already exist. Overwriting by {type(wrapped_class).__name__}"
                )
            cls.registry[project_name][calibrator_name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def create_calibrator(cls, project_name: str, calibrator_name: str, **kwargs) -> CalibratorBase:
        """
        Create the executor using a factory pattern.

        This method gets the appropriate Executor class from the registry
        and creates an instance of it, while passing in the parameters
        given in ``kwargs``.

        Args:
            name (str): The name of the executor to create.

        Returns
        -------
            An instance of the executor that is created.

        """
        if project_name not in cls.registry or calibrator_name not in cls.registry[project_name]:
            cls.logger.error(
                f"Calibrator project={project_name} name={calibrator_name} does not exist"
            )
            return None

        exec_class = cls.registry[project_name][calibrator_name]
        executor = exec_class(**kwargs)
        return executor
