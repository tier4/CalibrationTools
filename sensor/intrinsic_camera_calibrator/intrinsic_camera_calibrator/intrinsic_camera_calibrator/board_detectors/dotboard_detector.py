import cv2
from intrinsic_camera_calibrator.board_detections.dotboard_detection import DotBoardDetection
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.utils import to_grayscale
import numpy as np


class DotBoardDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.dotboard_int_param = Parameter(int, value=1, min_value=1, max_value=4)
        self.dotboard_float_param = Parameter(float, value=0.0, min_value=0.0, max_value=1.0)
        self.dotboard_bool_param = Parameter(bool, value=False, min_value=False, max_value=True)

        self.symmmetric_grid = Parameter(bool, value=True, min_value=False, max_value=True)
        self.clustering = Parameter(bool, value=True, min_value=False, max_value=True)

        self.filter_by_area = Parameter(bool, value=True, min_value=False, max_value=True)
        self.min_area_percentage = Parameter(float, value=0.01, min_value=0.001, max_value=0.1)
        self.max_area_percentage = Parameter(float, value=1.2, min_value=0.1, max_value=10.0)
        self.min_dist_between_blobs_percentage = Parameter(
            float, value=1.0, min_value=0.1, max_value=10.0
        )
        pass

    def detect(self, img: np.array):

        if img is None:
            self.detection_results_signal.emit(None, None)
            return

        # print(f"{threading.get_ident()} -> threaded detect: start")

        with self.lock:
            h, w = img.shape[0:2]
            (cols, rows) = (self.board_parameters.cols.value, self.board_parameters.rows.value)
            cell_size = self.board_parameters.cell_size.value

            # Setting blob detector
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea = self.filter_by_area.value
            params.minArea = self.min_area_percentage.value * h * w / 100.0
            params.maxArea = self.max_area_percentage.value * h * w / 100.0
            params.minDistBetweenBlobs = (
                self.min_dist_between_blobs_percentage.value * max(h, w) / 100.0
            )

            # params.minArea = 250
            # params.maxArea = 30000
            # params.minDistBetweenBlobs = 20
            # params.filterByConvexity = False
            # params.minCircularity = 0.1
            # params.filterByInertia = True
            # params.minInertiaRatio = 0.01

            detector = cv2.SimpleBlobDetector_create(params)

            # Visualize blob detection
            # keypoints = detector.detect(mono)
            # mono_with_keypoints = cv2.drawKeypoints(
            #     mono, keypoints, numpy.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # cv2.imshow("blob", mono_with_keypoints)

            flags = 0
            flags |= cv2.CALIB_CB_CLUSTERING if self.clustering.value else 0
            flags |= (
                cv2.CALIB_CB_SYMMETRIC_GRID
                if self.symmmetric_grid.value
                else cv2.CALIB_CB_ASYMMETRIC_GRID
            )

        # flags = cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING

        grayscale = to_grayscale(img)
        grayscale = np.array(grayscale)  # Is this really needed (?)

        (ok, corners) = cv2.findCirclesGrid(
            grayscale, (cols, rows), flags=flags, blobDetector=detector
        )

        # In symmetric case, findCirclesGrid does not detect the target if it's turned sideways. So we try
        # again with dimensions swapped - not so efficient.
        # TODO Better to add as second board? Corner ordering will change.
        if not ok:
            (ok, corners) = cv2.findCirclesGrid(
                grayscale, (cols, rows), flags=flags, blobDetector=detector
            )

            # We need to swap the axes of the detections back to make it consistent
            if ok:
                corners_2d_array = corners.reshape((cols, rows, 2))
                corners_transposed = np.transpose(corners_2d_array, (1, 0, 2))
                corners = corners_transposed.reshape(-1, 1, 2)

        if not ok:
            self.detection_results_signal.emit(img, None)
            return

        # cv2.drawChessboardCorners(img, (cols, rows), corners, ok)

        # reverse the corners if needed
        if np.linalg.norm(corners[0]) > np.linalg.norm(corners[1]):
            corners = np.flip(corners, axis=0)

        image_points = corners.reshape((rows, cols, 2))
        xarray = cell_size * (np.array(range(cols)) - 0.5 * cols)
        yarray = cell_size * (np.array(range(rows)) - 0.5 * rows)
        object_points = np.stack([*np.meshgrid(xarray, yarray), np.zeros((rows, cols))], axis=-1)

        detection = DotBoardDetection(
            height=h,
            width=w,
            rows=rows,
            cols=cols,
            object_points=object_points,
            image_points=image_points,
        )

        # print(f"{threading.get_ident()} -> threaded detect: end")
        self.detection_results_signal.emit(img, detection)
