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


import copy
import logging
import threading

from PySide2.QtCore import QObject
from PySide2.QtCore import QPoint
from PySide2.QtCore import QRectF
from PySide2.QtCore import QSize
from PySide2.QtCore import QThread
from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QColor
from PySide2.QtGui import QPainter
from PySide2.QtGui import QPen
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QGraphicsItem
from PySide2.QtWidgets import QGraphicsView
import cv2
import matplotlib.pyplot as plt
import numpy as np
from tier4_calibration_views.utils import decompose_transformation_matrix
from tier4_calibration_views.utils import transform_points


def intensity_to_rainbow_qcolor(value, alpha=1.0):
    h = value * 5.0 + 1.0
    i = h // 1  # floor(h)
    f = h - i
    if i % 2 == 0:
        f = 1.0 - f

    n = 1 - f

    if i <= 1:
        r, g, b = n, 0, 1.0
    elif i == 2:
        r, g, b = 0.0, n, 1.0
    elif i == 3:
        r, g, b = 0.0, 1.0, n
    elif i == 4:
        r, g, b = n, 1.0, 0.0
    elif i >= 5:
        r, g, b = 1.0, n, 0

    return QColor(int(255 * r), int(255 * g), int(255 * b), int(255 * alpha))


class RenderingData:
    def __init__(self):
        self.image_to_lidar_transform = None
        self.image_to_lidar_translation = None
        self.image_to_lidar_rotation = None

        self.draw_calibration_points_flag = False
        self.draw_pointcloud_flag = False
        self.draw_inliers_flag = False
        self.marker_size_pixels = None
        self.marker_size_meters = None
        self.color_channel = None
        self.marker_units = None
        self.marker_type = None
        self.rendering_alpha = None
        self.subsample_factor = None
        self.rainbow_distance = None
        self.rainbow_offset = 0
        self.min_rendering_distance = 0.0
        self.max_rendering_distance = 100.0

        self.current_object_point = None
        self.object_points = None
        self.image_points = None
        self.external_object_points = None
        self.external_image_points = None
        self.pointcloud_xyz = None
        self.pointcloud_intensity = None

        self.widget_size = None

        self.k = None
        self.d = None


class CustomQGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(CustomQGraphicsView, self).__init__(parent)

    def resizeEvent(self, event):
        super().resizeEvent(event)

        for item in self.scene().items():
            item.prepareGeometryChange()
            item.update()

    def wheelEvent(self, event):
        zoom_in_factor = 1.25
        zoom_out_factor = 1 / zoom_in_factor

        for item in self.scene().items():
            item.prepareGeometryChange()
            item.update()

        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

        old_pos = self.mapToScene(event.pos())

        # Zoom
        if event.delta() > 0:
            zoom_factor = zoom_in_factor
        else:
            zoom_factor = zoom_out_factor
        self.scale(zoom_factor, zoom_factor)

        # Get the new position
        new_pos = self.mapToScene(event.pos())

        # Move scene to old position
        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())


class Renderer(QObject):
    def __init__(self, image_view):
        super().__init__()
        self.image_view = image_view

    def render(self):
        self.image_view.paintEventThread()


class ImageView(QGraphicsItem, QObject):
    clicked_signal = Signal(float, float)
    render_request_signal = Signal()
    rendered_signal = Signal()

    def __init__(self, parent=None):
        QGraphicsItem.__init__(self, parent)
        QObject.__init__(self, parent)

        self.pix = QPixmap()
        self.image_width = None
        self.image_height = None
        self.display_width = None
        self.display_height = None

        self.data_ui = RenderingData()
        self.data_renderer = RenderingData()

        self.thread = QThread()
        self.thread.start()
        self.lock = threading.RLock()
        self.renderer = Renderer(self)
        self.renderer.moveToThread(self.thread)
        # To debug rendering, consider not moving it into another thread

        self.rendered_signal.connect(self.update2)
        self.render_request_signal.connect(self.renderer.render)

        self.line_pen = QPen()
        self.line_pen.setWidth(2)
        self.line_pen.setBrush(Qt.white)

        self.magenta_pen = QPen()
        self.magenta_pen.setWidth(2)
        self.magenta_pen.setBrush(Qt.magenta)

        self.cyan_pen = QPen()
        self.cyan_pen.setWidth(2)
        self.cyan_pen.setBrush(Qt.cyan)

        self.inlier_line_pen = QPen()
        self.inlier_line_pen.setWidth(2)
        self.inlier_line_pen.setBrush(Qt.green)

        self.outlier_line_pen = QPen()
        self.outlier_line_pen.setWidth(2)
        self.outlier_line_pen.setBrush(Qt.red)

        self.red_pen = QPen(Qt.red)
        self.red_pen.setBrush(Qt.red)

        self.green_pen = QPen(Qt.green)
        self.green_pen.setBrush(Qt.green)

        colormap_name = "hsv"
        self.colormap_bins = 100
        self.colormap = plt.get_cmap(colormap_name, self.colormap_bins)
        self.colormap = [
            (
                int(255 * self.colormap(i)[2]),
                int(255 * self.colormap(i)[1]),
                int(255 * self.colormap(i)[0]),
            )
            for i in range(self.colormap_bins)
        ]

        # self.setMinimumWidth(300)

        self.update_count = 0
        self.render_count = 0
        self.unprocessed_rendered_requests = 0
        self.rendering = False
        self.rendered_image = None

    def update(self):
        with self.lock:
            self.update_count += 1
            self.unprocessed_rendered_requests += 1
        self.render_request_signal.emit()

    def update2(self):
        super().update()

    def set_draw_calibration_points(self, value):
        with self.lock:
            self.data_ui.draw_calibration_points_flag = value
        self.update()

    def set_draw_pointcloud(self, value):
        with self.lock:
            self.data_ui.draw_pointcloud_flag = value
        self.update()

    def set_marker_size_pixels(self, value):
        with self.lock:
            self.data_ui.marker_size_pixels = value
        self.update()

    def set_marker_size_meters(self, value):
        with self.lock:
            self.data_ui.marker_size_meters = value
        self.update()

    def set_rainbow_distance(self, value):
        with self.lock:
            self.data_ui.rainbow_distance = value
        self.update()

    def set_rainbow_offset(self, value):
        with self.lock:
            self.data_ui.rainbow_offset = value
        self.update()

    def set_rendering_alpha(self, value):
        with self.lock:
            self.data_ui.rendering_alpha = value
        self.update()

    def set_marker_type(self, value):
        with self.lock:
            self.data_ui.marker_type = value.lower()
        self.update()

    def set_marker_units(self, value):
        with self.lock:
            value = value.lower()
            assert value == "meters" or value == "pixels"
            self.data_ui.marker_units = value
        self.update()

    def set_color_channel(self, value):
        with self.lock:
            self.data_ui.color_channel = value.lower()
        self.update()

    def set_draw_inliers(self, value):
        with self.lock:
            self.data_ui.draw_inliers_flag = value
        self.update()

    def set_inlier_distance(self, value):
        with self.lock:
            self.data_ui.inlier_distance = value
        self.update()

    def set_min_rendering_distance(self, value):
        with self.lock:
            self.data_ui.min_rendering_distance = value
        self.update()

    def set_max_rendering_distance(self, value):
        with self.lock:
            self.data_ui.max_rendering_distance = value
        self.update()

    def set_current_point(self, point):
        with self.lock:
            self.data_ui.current_object_point = None if point is None else point.reshape(1, 3)
        self.update()

    def set_transform(self, transform):
        with self.lock:
            self.data_ui.image_to_lidar_transform = transform
            (
                self.data_ui.image_to_lidar_translation,
                self.data_ui.image_to_lidar_rotation,
            ) = decompose_transformation_matrix(transform)
        self.update()

    def pixmap(self):
        with self.lock:
            return self.pix

    def set_pixmap(self, pixmap):
        with self.lock:
            if self.pix is None or self.pix.size() != pixmap.size():
                self.prepareGeometryChange()

            self.pix = pixmap

            self.image_width = float(self.pix.size().width())
            self.image_height = float(self.pix.size().height())

    def set_subsample_factor(self, value):
        with self.lock:
            self.data_ui.subsample_factor = int(value)
        self.update()

    def set_pointcloud(self, pointcloud):
        with self.lock:
            self.data_ui.pointcloud_xyz = pointcloud[:, 0:3]
            self.data_ui.pointcloud_intensity = pointcloud[:, 3]

            subsample = self.data_ui.subsample_factor

            self.data_ui.pointcloud_xyz = self.data_ui.pointcloud_xyz[::subsample, :]
            self.data_ui.pointcloud_intensity = self.data_ui.pointcloud_intensity[::subsample]

    def set_camera_info(self, k, d):
        with self.lock:
            self.data_ui.k = np.copy(k).reshape((3, 3))
            self.data_ui.d = np.copy(d).reshape((-1,))

    def set_calibration_points(self, object_points, image_points):
        with self.lock:
            self.data_ui.object_points = object_points
            self.data_ui.image_points = image_points
        self.update()

    def set_external_calibration_points(self, object_points, image_points):
        with self.lock:
            self.data_ui.external_object_points = object_points
            self.data_ui.external_image_points = image_points
        self.update()

    def minimumSizeHint(self):
        return QSize(1000, 400)

    def sizeHint(self):
        return QSize(1000, 1000)

    def take_screenshot(self):
        with self.lock:
            return self.rendered_image.copy()

    def paint(self, painter, option, widget):
        with self.lock:
            self.data_ui.widget_size = widget.size()
            painter.setRenderHint(QPainter.Antialiasing)
            painter.drawPixmap(QPoint(), self.rendered_image)

    def boundingRect(self):
        with self.lock:
            if self.rendered_image is None:
                return QRectF(0, 0, 500, 500)

            return QRectF(
                0, 0, self.rendered_image.size().width(), self.rendered_image.size().height()
            )

    def paintEventThread(self):
        with self.lock:
            self.render_count += 1
            self.unprocessed_rendered_requests -= 1

            if self.pix.isNull():
                return

            if self.unprocessed_rendered_requests > 0:
                self.rendered.emit()
                return

            # Copy the data into the thread
            self.data_renderer = copy.deepcopy(self.data_ui)

            if self.data_renderer.widget_size is None:
                return

            scaled_pix_size = self.pix.size()
            scaled_pix_size.scale(self.data_renderer.widget_size, Qt.KeepAspectRatio)

            rendered_image = self.pix.scaled(
                scaled_pix_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )

        painter = QPainter(rendered_image)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(Qt.red)

        painter.drawLine(QPoint(0, 0), QPoint(0, scaled_pix_size.height()))
        painter.drawLine(
            QPoint(0, scaled_pix_size.height()),
            QPoint(scaled_pix_size.width(), scaled_pix_size.height()),
        )
        painter.drawLine(
            QPoint(scaled_pix_size.width(), scaled_pix_size.height()),
            QPoint(scaled_pix_size.width(), 0),
        )
        painter.drawLine(QPoint(scaled_pix_size.width(), 0), QPoint(0, 0))

        self.width_image_to_widget_factor = float(scaled_pix_size.width()) / self.image_width
        self.height_image_to_widget_factor = float(scaled_pix_size.height()) / self.image_height
        self.image_to_widget_factor = np.array(
            [self.width_image_to_widget_factor, self.height_image_to_widget_factor]
        )

        if self.data_renderer.draw_pointcloud_flag:
            self.draw_pointcloud(painter)

        if self.data_renderer.draw_calibration_points_flag:
            self.draw_calibration_points(painter)
            self.draw_external_calibration_points(painter)

        self.draw_current_point(painter)

        painter.end()

        with self.lock:
            self.rendered_image = rendered_image
            self.rendered_signal.emit()

    def draw_pointcloud(self, painter):
        if (
            self.data_renderer.image_to_lidar_translation is None
            or self.data_renderer.image_to_lidar_rotation is None
        ):
            return

        # Note: ccs=camera coordinate system
        pointcloud_ccs = transform_points(
            self.data_renderer.image_to_lidar_translation,
            self.data_renderer.image_to_lidar_rotation,
            self.data_renderer.pointcloud_xyz,
        )

        if pointcloud_ccs.shape[0] == 0:
            return

        # Transform to the image coordinate system
        tvec = np.zeros((3, 1))
        rvec = np.zeros((3, 1))

        pointcloud_ics, _ = cv2.projectPoints(
            pointcloud_ccs, rvec, tvec, self.data_renderer.k, self.data_renderer.d
        )

        pointcloud_ics = pointcloud_ics.reshape(-1, 2)

        indexes = np.logical_and(
            np.logical_and(
                np.logical_and(pointcloud_ics[:, 0] >= 0, pointcloud_ics[:, 0] < self.image_width),
                np.logical_and(pointcloud_ics[:, 1] >= 0, pointcloud_ics[:, 1] < self.image_height),
            ),
            np.logical_and(
                pointcloud_ccs[:, 2] >= self.data_renderer.min_rendering_distance,
                pointcloud_ccs[:, 2] < self.data_renderer.max_rendering_distance,
            ),
        )

        # Transform (rescale) into the widget coordinate system
        pointcloud_z = pointcloud_ccs[indexes, 2]
        pointcloud_i = self.data_renderer.pointcloud_intensity[indexes]

        if self.data_renderer.marker_units == "meters":
            factor = (
                self.data_renderer.k[0, 0]
                * self.data_renderer.marker_size_meters
                * self.width_image_to_widget_factor
            )
            scale_px = factor / pointcloud_z
        else:
            factor = self.data_renderer.marker_size_pixels * self.width_image_to_widget_factor
            scale_px = factor * np.ones_like(pointcloud_z)

        pointcloud_wcs = pointcloud_ics[indexes, :] * self.image_to_widget_factor

        indexes2 = scale_px >= 1
        pointcloud_wcs = pointcloud_wcs[indexes2, :]
        scale_px = scale_px[indexes2]

        if pointcloud_wcs.shape[0] == 0:
            return

        try:
            if self.data_renderer.color_channel == "x":
                color_scalars = pointcloud_ccs[indexes, 0][indexes2]
            elif self.data_renderer.color_channel == "y":
                color_scalars = pointcloud_ccs[indexes, 1][indexes2]
            elif self.data_renderer.color_channel == "z":
                color_scalars = pointcloud_z[indexes2]
            elif self.data_renderer.color_channel == "intensity":
                color_scalars = pointcloud_i[indexes2]
                min_value = color_scalars.min()
                max_value = color_scalars.max()
                if min_value == max_value:
                    color_scalars = np.ones_like(color_scalars)
                else:
                    color_scalars = 1.0 - (color_scalars - min_value) / (max_value - min_value)
            else:
                raise NotImplementedError
        except Exception as e:
            logging.error(e)

        line_pen = QPen()
        line_pen.setWidth(2)
        line_pen.setBrush(Qt.white)

        painter.setPen(Qt.blue)
        painter.setBrush(Qt.blue)

        draw_marker_f = (
            painter.drawEllipse if self.data_renderer.marker_type == "circles" else painter.drawRect
        )

        # print(f"Drawing pointcloud size: {scale_px.shape[0]}")

        for point, radius, color_channel in zip(pointcloud_wcs, scale_px, color_scalars):
            if self.data_renderer.color_channel == "intensity":
                color = intensity_to_rainbow_qcolor(
                    color_channel, self.data_renderer.rendering_alpha
                )
            else:
                color_index = int(
                    self.colormap_bins
                    * (
                        (
                            self.data_renderer.rainbow_offset
                            + (color_channel / self.data_renderer.rainbow_distance)
                        )
                        % 1.0
                    )
                )
                color = self.colormap[color_index]
                color = QColor(
                    color[0], color[1], color[2], int(255 * self.data_renderer.rendering_alpha)
                )

            painter.setPen(color)
            painter.setBrush(color)
            draw_marker_f(point[0] - 0.5 * radius, point[1] - 0.5 * radius, radius, radius)

    def draw_calibration_points(self, painter):
        if (
            self.data_renderer.image_points is None
            or self.data_renderer.object_points is None
            or len(self.data_renderer.image_points) == 0
            or len(self.data_renderer.object_points) == 0
            or self.data_renderer.image_to_lidar_translation is None
            or self.data_renderer.image_to_lidar_rotation is None
        ):
            return

        image_points = np.array(self.data_renderer.image_points)

        # Note: lcs=lidar coordinate system
        object_points_lcs = np.array(self.data_renderer.object_points)

        # Note: ccs=camera coordinate system
        object_points_ccs = transform_points(
            self.data_renderer.image_to_lidar_translation,
            self.data_renderer.image_to_lidar_rotation,
            object_points_lcs,
        )

        # Transform to the image coordinate system
        tvec = np.zeros((3, 1))
        rvec = np.zeros((3, 1))
        object_points_ics, _ = cv2.projectPoints(
            object_points_ccs, rvec, tvec, self.data_renderer.k, self.data_renderer.d
        )
        object_points_ics = object_points_ics.reshape(-1, 2)

        repr_err = np.linalg.norm(object_points_ics - image_points, axis=1)

        # Transform (rescale) into the widget coordinate system
        object_points_wcs = object_points_ics * self.image_to_widget_factor

        radius = 10 * self.width_image_to_widget_factor

        object_pen = self.red_pen
        image_pen = self.green_pen
        line_pen = self.line_pen

        for object_point_wcs, image_point, d in zip(
            object_points_wcs, self.data_renderer.image_points, repr_err
        ):
            image_point_wcs = image_point * self.image_to_widget_factor

            if self.data_renderer.draw_inliers_flag:
                if d < self.data_renderer.inlier_distance:
                    line_pen = self.inlier_line_pen
                    object_pen = self.green_pen
                    image_pen = self.green_pen
                else:
                    line_pen = self.outlier_line_pen
                    object_pen = self.red_pen
                    image_pen = self.red_pen

            painter.setPen(line_pen)
            painter.drawLine(
                object_point_wcs[0], object_point_wcs[1], image_point_wcs[0], image_point_wcs[1]
            )

            painter.setPen(object_pen)
            painter.setBrush(object_pen.brush())
            painter.drawEllipse(
                object_point_wcs[0] - 0.5 * radius,
                object_point_wcs[1] - 0.5 * radius,
                radius,
                radius,
            )

            painter.setPen(image_pen)
            painter.setBrush(image_pen.brush())
            painter.drawEllipse(
                image_point_wcs[0] - 0.5 * radius, image_point_wcs[1] - 0.5 * radius, radius, radius
            )

    def draw_external_calibration_points(self, painter):
        if (
            self.data_renderer.external_image_points is None
            or self.data_renderer.external_object_points is None
            or len(self.data_renderer.external_image_points) == 0
            or len(self.data_renderer.external_object_points) == 0
            or self.data_renderer.image_to_lidar_translation is None
            or self.data_renderer.image_to_lidar_rotation is None
        ):
            return

        # Note: lcs=lidar coordinate system, ccs=camera coordinate system. ics=image coordinate system
        object_points_lcs = np.array(self.data_renderer.external_object_points)

        object_points_ccs = transform_points(
            self.data_renderer.image_to_lidar_translation,
            self.data_renderer.image_to_lidar_rotation,
            object_points_lcs,
        )

        # Transform to the image coordinate system
        tvec = np.zeros((3, 1))
        rvec = np.zeros((3, 1))
        object_points_ics, _ = cv2.projectPoints(
            object_points_ccs, rvec, tvec, self.data_renderer.k, self.data_renderer.d
        )
        object_points_ics = object_points_ics.reshape(-1, 2)

        # Transform (rescale) into the widget coordinate system
        object_points_wcs = object_points_ics * self.image_to_widget_factor

        radius = 10 * self.width_image_to_widget_factor

        object_pen = self.red_pen
        image_pen = self.green_pen
        line_pen = self.line_pen
        object_line_pen = self.magenta_pen
        image_line_pen = self.cyan_pen

        # Draw tag borders
        image_points = self.data_renderer.external_image_points

        scaled_pix_size = self.pix.size()
        scaled_pix_size.scale(self.data_renderer.widget_size, Qt.KeepAspectRatio)

        for i1 in range(len(image_points)):
            tag_index = i1 // 4
            i2 = 4 * tag_index + ((i1 + 1) % 4)

            image_point_1_wcs = image_points[i1] * self.image_to_widget_factor
            image_point_2_wcs = image_points[i2] * self.image_to_widget_factor

            object_point_1_wcs = object_points_wcs[i1]
            object_point_2_wcs = object_points_wcs[i2]

            painter.setPen(image_line_pen)
            painter.drawLine(
                image_point_1_wcs[0],
                image_point_1_wcs[1],
                image_point_2_wcs[0],
                image_point_2_wcs[1],
            )

            if (
                np.any(np.isnan(object_point_1_wcs))
                or np.any(np.isnan(object_point_2_wcs))
                or object_point_1_wcs[0] < 0
                or object_point_1_wcs[0] > scaled_pix_size.width()
                or object_point_1_wcs[1] < 0
                or object_point_1_wcs[1] > scaled_pix_size.height()
                or object_point_2_wcs[0] < 0
                or object_point_2_wcs[0] > scaled_pix_size.width()
                or object_point_2_wcs[1] < 0
                or object_point_2_wcs[1] > scaled_pix_size.height()
            ):
                continue

            painter.setPen(object_line_pen)
            painter.drawLine(
                object_point_1_wcs[0],
                object_point_1_wcs[1],
                object_point_2_wcs[0],
                object_point_2_wcs[1],
            )

        # Draw normal points
        for object_point_wcs, image_point in zip(
            object_points_wcs, self.data_renderer.external_image_points
        ):
            image_point_wcs = image_point * self.image_to_widget_factor

            painter.setPen(line_pen)
            painter.drawLine(
                object_point_wcs[0], object_point_wcs[1], image_point_wcs[0], image_point_wcs[1]
            )

            painter.setPen(object_pen)
            painter.setBrush(object_pen.brush())
            painter.drawEllipse(
                object_point_wcs[0] - 0.5 * radius,
                object_point_wcs[1] - 0.5 * radius,
                radius,
                radius,
            )

            painter.setPen(image_pen)
            painter.setBrush(image_pen.brush())
            painter.drawEllipse(
                image_point_wcs[0] - 0.5 * radius, image_point_wcs[1] - 0.5 * radius, radius, radius
            )

    def draw_current_point(self, painter):
        if self.data_renderer.current_object_point is None:
            return

        if (
            self.data_renderer.image_to_lidar_translation is None
            or self.data_renderer.image_to_lidar_rotation is None
        ):
            return

        # Note: wcs=widget coordinate system, ccs=camera coordinate system. ics=image coordinate system
        object_point_ccs = transform_points(
            self.data_renderer.image_to_lidar_translation,
            self.data_renderer.image_to_lidar_rotation,
            self.data_renderer.current_object_point,
        )

        # Transform to the image coordinate system
        tvec = np.zeros((3, 1))
        rvec = np.zeros((3, 1))
        object_point_ics, _ = cv2.projectPoints(
            object_point_ccs, rvec, tvec, self.data_renderer.k, self.data_renderer.d
        )
        object_point_ics = object_point_ics.reshape(1, 2)

        # Transform (rescale) into the widget coordinate system
        object_point_wcs = object_point_ics * self.image_to_widget_factor
        object_point_wcs = object_point_wcs.reshape(
            2,
        )

        radius = 20 * self.width_image_to_widget_factor

        painter.setPen(Qt.magenta)
        painter.drawLine(
            object_point_wcs[0] - radius,
            object_point_wcs[1] - radius,
            object_point_wcs[0] + radius,
            object_point_wcs[1] + radius,
        )

        painter.drawLine(
            object_point_wcs[0] + radius,
            object_point_wcs[1] - radius,
            object_point_wcs[0] - radius,
            object_point_wcs[1] + radius,
        )

    def mousePressEvent(self, e):
        with self.lock:
            if self.pix is None or self.data_renderer.widget_size is None:
                return

            scaled_pix_size = self.pix.size()
            scaled_pix_size.scale(self.data_renderer.widget_size, Qt.KeepAspectRatio)

        width_widget_to_image_factor = self.image_width / float(scaled_pix_size.width())
        height_widget_to_image_factor = self.image_height / float(scaled_pix_size.height())

        x = (e.scenePos().x() + 0.5) * width_widget_to_image_factor
        y = (e.scenePos().y() + 0.5) * height_widget_to_image_factor

        if x >= 0 and x < self.image_width and y >= 0 and y < self.image_height:
            self.update()

            self.clicked_signal.emit(x, y)
        else:
            logging.error("Click out of image coordinates !")

        self.prepareGeometryChange()
        return super().mousePressEvent(e)
