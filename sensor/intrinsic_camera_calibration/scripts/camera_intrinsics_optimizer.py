#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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

import array
import cv2
import nlopt
import numpy as np
import rclpy

from copy import deepcopy
from rclpy.node import Node
from tier4_calibration_msgs.srv import IntrinsicsOptimizer

class CameraIntrinsicsOptimizer(Node):
    def __init__(self):

        super().__init__("camera_intrinsics_optimizer")

        self.object_points = None
        self.image_points = None
        self.k0 = None
        self.iteration = 0

        self.declare_parameter("opt_method", "CV")
        self.declare_parameter("opt_scale", 0.15)
        self.declare_parameter("k_coefficients", 2)
        self.declare_parameter("fix_principal_point", False)
        self.declare_parameter("fix_aspect_ratio", False)
        self.declare_parameter("zero_tangent_dist", False)

        self.method = self.get_parameter("opt_method").get_parameter_value().string_value
        self.opt_allowed_percentage = self.get_parameter("opt_scale")
        self.opt_allowed_percentage = self.opt_allowed_percentage.get_parameter_value().double_value

        num_ks = self.get_parameter("k_coefficients").get_parameter_value().integer_value
        fix_principal_point = self.get_parameter("fix_principal_point")
        fix_principal_point = fix_principal_point.get_parameter_value().bool_value
        fix_aspect_ratio = self.get_parameter("fix_aspect_ratio")
        fix_aspect_ratio = fix_aspect_ratio.get_parameter_value().bool_value
        zero_tangent_dist = self.get_parameter("zero_tangent_dist")
        zero_tangent_dist = zero_tangent_dist.get_parameter_value().bool_value

        self.calib_flags = 0

        if fix_principal_point:
            self.calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        if fix_aspect_ratio:
            self.calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
        if zero_tangent_dist:
            self.calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
        if (num_ks > 3):
            self.calib_flags |= cv2.CALIB_RATIONAL_MODEL
        if (num_ks < 6):
            self.calib_flags |= cv2.CALIB_FIX_K6
        if (num_ks < 5):
            self.calib_flags |= cv2.CALIB_FIX_K5
        if (num_ks < 4):
            self.calib_flags |= cv2.CALIB_FIX_K4
        if (num_ks < 3):
            self.calib_flags |= cv2.CALIB_FIX_K3
        if (num_ks < 2):
            self.calib_flags |= cv2.CALIB_FIX_K2
        if (num_ks < 1):
            self.calib_flags |= cv2.CALIB_FIX_K1
        self.calib_flags |= cv2.CALIB_USE_INTRINSIC_GUESS

        if self.method == "CV":
            pass
        elif self.method == "LN_COBYLA":
            self.method = nlopt.LN_COBYLA
        elif self.method == "LN_SBPLX":
            self.method = nlopt.LN_SBPLX
        elif self.method == "GN_CRS2_LM":
            self.method = nlopt.GN_CRS2_LM
        else:
            raise NotImplementedError

        # Advertise service
        self.opt_serice = self.create_service(IntrinsicsOptimizer,
            "optimize_intrinsics", self.service_callback)

    def reproj_error(self, object_points, image_points, k):

        d = np.zeros((5,))
        k = np.reshape(k, (3,3))

        _, rvec, tvec = cv2.solvePnP(
            object_points, image_points, k, d, flags=cv2.SOLVEPNP_SQPNP)

        num_points, dim = object_points.shape
        projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
        projected_points = projected_points.reshape((num_points, 2))
        reproj_error = np.linalg.norm(projected_points - image_points, axis=1).mean()

        return reproj_error

    def param_to_k(self, params):

        k_opt = np.eye(3)
        k_opt[0,0] = self.fx0 + self.opt_allowed_percentage*self.fx0*params[0]
        k_opt[1,1] = self.fy0 + self.opt_allowed_percentage*self.fy0*params[1]
        k_opt[0,2] = self.cx0 + self.opt_allowed_percentage*self.cx0*params[2]
        k_opt[1,2] = self.cy0 + self.opt_allowed_percentage*self.cy0*params[3]

        return k_opt

    def opt_f(self, args):

        k_opt = self.param_to_k(args)

        error = self.reproj_error(self.object_points, self.image_points, k_opt)

        if self.iteration % 100 == 0:
            self.get_logger().info(f"iteration={self.iteration} -> error: {error}")

        self.iteration += 1

        return error

    def optimize_nlopt(self, object_points, image_points, initial_camera_info):

        self.object_points = object_points
        self.image_points = image_points

        initial_k = np.array(initial_camera_info.k).reshape(3, 3)
        initial_d = np.array(initial_camera_info.d).flatten()

        if (np.abs(initial_d).sum() != 0.0):
            self.get_logger().error("We only support distortion-less intrinsics for now")
            return initial_camera_info

        self.k0 = np.array(initial_k)
        self.fx0 = self.k0[0, 0]
        self.fy0 = self.k0[1, 1]
        self.cx0 = self.k0[0, 2]
        self.cy0 = self.k0[1, 2]

        self.iteration = 0

        x0 = [0.0, 0.0, 0.0, 0.0]
        opt = nlopt.opt(nlopt.LN_SBPLX, 4)

        def f(x, grad):
            return float(self.opt_f(x))

        opt.set_min_objective(f)
        opt.set_lower_bounds([-1.0, -1.0, -1.0, -1.0])
        opt.set_upper_bounds([1.0, 1.0, 1.0, 1.0])

        tol = 1e-12
        opt.set_ftol_abs(tol)
        opt.set_xtol_rel(np.sqrt(tol))

        opt.set_maxeval(100000)
        params = opt.optimize(x0)
        optimized_k = self.param_to_k(params)

        self.get_logger().info("Optimization result")
        self.get_logger().info(f"Parameters: {params}")
        self.get_logger().info(f"Initial eval: {self.opt_f(x0)}")
        self.get_logger().info(f"Final eval: {self.opt_f(params)}")
        self.get_logger().info(f"Num evals: {opt.get_numevals()}")
        self.get_logger().info(f"Initial K:\n {self.k0}")
        self.get_logger().info(f"Final K:\n {optimized_k}")

        optimized_camera_info = deepcopy(initial_camera_info)
        optimized_camera_info.k = optimized_k.reshape(-1)

        return optimized_camera_info

    def optimize_cv(self, object_points, image_points, initial_camera_info):

        initial_k = np.array(initial_camera_info.k).reshape(3, 3)
        initial_d = np.array(initial_camera_info.d).flatten()

        _, new_k, new_d, _, _ = cv2.calibrateCamera(
            [object_points.reshape(-1,3)],
            [image_points.reshape(-1,1,2)],
            (initial_camera_info.width, initial_camera_info.height),
            cameraMatrix=initial_k, distCoeffs=initial_d, flags=self.calib_flags)

        optimized_camera_info = deepcopy(initial_camera_info)
        optimized_camera_info.k = new_k.reshape(-1)
        optimized_camera_info.d = array.array('d', new_d)

        return optimized_camera_info

    def service_callback(self,
                         request : IntrinsicsOptimizer.Request,
                         response: IntrinsicsOptimizer.Response):

        points = request.calibration_points
        initial_camera_info = request.initial_camera_info

        object_points = np.array([np.array([p.x, p.y, p.z]) for p in points.object_points])
        object_points = object_points.astype(np.float32)

        image_points = np.array([np.array([p.x, p.y]) for p in points.image_points])
        image_points = image_points.astype(np.float32)

        num_object_points, object_dim = object_points.shape
        num_image_points, image_dim = image_points.shape

        assert num_object_points == num_image_points
        assert object_dim == 3
        assert image_dim == 2

        if self.method == "CV":
            optimized_camera_info = self.optimize_cv(
                object_points, image_points, initial_camera_info)
        else:
            optimized_camera_info = self.optimize_nlopt(
                object_points, image_points, initial_camera_info)

        ncm, _ = cv2.getOptimalNewCameraMatrix(
            np.array(optimized_camera_info.k).reshape(3, 3),
            np.array(optimized_camera_info.d).reshape(-1),
            (optimized_camera_info.width, optimized_camera_info.height), 0.0)

        p = np.zeros((3, 4), dtype=np.float64)

        for j in range(3):
            for i in range(3):
                p[j,i] = ncm[j, i]

        optimized_camera_info.p = p.reshape(-1)
        response.optimized_camera_info = optimized_camera_info

        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CameraIntrinsicsOptimizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
