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

from typing import List

from PySide2.QtCore import QObject
from PySide2.QtCore import QPointF
from PySide2.QtCore import QRectF
from PySide2.QtCore import QSize
from PySide2.QtCore import Qt
from PySide2.QtGui import QBrush
from PySide2.QtGui import QColor
from PySide2.QtGui import QImage
from PySide2.QtGui import QPainter
from PySide2.QtGui import QPen
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QGraphicsItem
from PySide2.QtWidgets import QGraphicsView
import cv2
import numpy as np


class CustomQGraphicsView(QGraphicsView):
    """Custom subclass of QGraphicsView to control ImageView and add zoom, pan, and other interactions."""

    def __init__(self, parent=None):
        super(CustomQGraphicsView, self).__init__(parent)

        self.is_pan_active = False
        self.pan_start_x = None
        self.pan_start_y = None

    def resizeEvent(self, event):
        super().resizeEvent(event)

        for item in self.scene().items():
            item.prepareGeometryChange()
            item.update()

    def mouseMoveEvent(self, event):
        if self.is_pan_active:
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - (event.x() - self.pan_start_x)
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - (event.y() - self.pan_start_y)
            )
            self.pan_start_x = event.x()
            self.pan_start_y = event.y()
            event.accept()
            return

        event.ignore()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_pan_active = True
            self.pan_start_x = event.x()
            self.pan_start_y = event.y()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        event.ignore()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_pan_active = False
            self.pan_start_x = None
            self.pan_start_y = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return

        event.ignore()

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


class ImageView(QGraphicsItem, QObject):
    """QGraphicsItem subclass to render the image and other useful statistics."""

    def __init__(self, parent=None):
        QGraphicsItem.__init__(self, parent)
        QObject.__init__(self, parent)

        self.raw_pixmap = QPixmap()
        self.display_pixmap = QPixmap()

        self.is_draw_detection_points = True
        self.detection_points_array = None
        self.training_heatmap = None
        self.evaluation_heatmap = None

        self.line_pen = QPen()
        self.line_pen.setWidth(2)
        self.line_pen.setBrush(Qt.white)

        self.magenta_pen = QPen()
        self.magenta_pen.setWidth(2)
        self.magenta_pen.setBrush(Qt.magenta)

        self.cyan_pen = QPen()
        self.cyan_pen.setWidth(2)
        self.cyan_pen.setBrush(Qt.cyan)

        self.red_pen = QPen(Qt.red)
        self.red_pen.setBrush(Qt.red)

        self.green_pen = QPen(Qt.green)
        self.green_pen.setBrush(Qt.green)

        self.draw_detection_color = QColor()

    def set_draw_detection_color(self, color: QColor):
        """Set the color in which to draw the center of the detections on."""
        self.draw_detection_color = color

    def set_draw_detection_points(self, value: bool):
        """Set the flag of wether to draw the detection points or not."""
        self.is_draw_detection_points = value

    def set_draw_training_points(self, value: bool):
        """Set the flag of wether or not to draw all the detection points present in the training dataset."""
        self.is_draw_training_points = value

    def set_draw_evaluation_points(self, value: bool):
        """Set the flag of wether or not to draw all the detection points present in the training dataset."""
        self.is_draw_evaluation_points = value

    def set_draw_training_heatmap(self, value: bool):
        """Set the flag of wether or not to draw the occupancy heatmap of the training dataset."""
        self.is_draw_training_heatmap = value

    def set_draw_evaluation_heatmap(self, value: bool):
        """Set the flag of wether or not to draw the occupancy heatmap of the evaluation dataset."""
        self.is_draw_evaluation_heatmap = value

    def set_detection_ordered_points(self, points_list: List[np.array]):
        """Set the detection points to draw. The points are expected to be in a list of rows, where each row is rendered in a different color."""
        self.points_list = points_list

    def set_training_points(self, value: np.array):
        """Set the detection points from the training dataset."""
        self.training_points = value

    def set_evaluation_points(self, value: np.array):
        """Set the detection points from the evaluation dataset."""
        self.evaluation_points = value

    def set_training_heatmap(self, value: np.array):
        """Set the occupancy heatmap to draw from the training dataset."""
        self.training_heatmap = value

    def set_evaluation_heatmap(self, value: np.array):
        """Set the occupancy heatmap to draw from the evaluation dataset."""
        self.evaluation_heatmap = value

    def set_grid_size_pixels(self, cell_size_pixels):
        """Set the size in which to draw the detection's corners."""
        self.cell_size_pixels = cell_size_pixels

    def set_rendering_alpha(self, value: float):
        """Set the alpha channel of the drawings."""
        self.rendering_alpha = value

    def pixmap(self) -> QPixmap:
        """Return the rendering QPixmap."""
        return self.raw_pixmap

    def set_image(self, img: np.array):
        """Set the image to render from an array to a QPixmap an resize the window if necessary."""
        height, width, channel = img.shape
        assert channel == 3
        bytes_per_line = channel * width
        q_img = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.raw_pixmap = QPixmap(q_img)

        if self.raw_pixmap is None or self.raw_pixmap.size() != self.raw_pixmap.size():
            self.prepareGeometryChange()

    def minimumSizeHint(self):
        """Return a size for Qt to infer the correct size of the view."""
        return QSize(1000, 400)

    def sizeHint(self):
        """Return a size for Qt to infer the correct size of the view."""
        return QSize(1000, 1000)

    def draw_detection_points(self, painter: QPainter):
        """
        Draw the detection's corners in the view.

        Each corner is drawn as a detection, and each row is connected through a line of the same color.
        """
        if self.points_list is None:
            return

        self.draw_detection_color.setAlphaF(np.sin(0.5 * np.pi * self.rendering_alpha))

        estimated_size = self.cell_size_pixels * self.image_to_view_factor.mean() / 8
        inv_sqrt2 = 1.0 / np.sqrt(2)

        alpha = int(255 * self.rendering_alpha)
        colors = [
            QColor(0, 0, 255, alpha),
            QColor(0, 128, 255, alpha),
            QColor(0, 200, 200, alpha),
            QColor(0, 255, 0, alpha),
            QColor(200, 200, 0, alpha),
            QColor(255, 0, 0, alpha),
            QColor(255, 0, 255, alpha),
        ]

        for i, points in enumerate(self.points_list):
            point_list = []
            for p in points:
                p = self.image_to_view_factor * p
                p = QPointF(p[0], p[1])
                point_list.append(p)

                pen = QPen(self.draw_detection_color)
                pen.setWidth(0.1)

                painter.setPen(pen)
                painter.drawEllipse(p, estimated_size, estimated_size)
                painter.drawLine(
                    p + QPointF(-estimated_size * inv_sqrt2, -estimated_size * inv_sqrt2),
                    p + QPointF(estimated_size * inv_sqrt2, estimated_size * inv_sqrt2),
                )
                painter.drawLine(
                    p + QPointF(estimated_size * inv_sqrt2, -estimated_size * inv_sqrt2),
                    p + QPointF(-estimated_size * inv_sqrt2, estimated_size * inv_sqrt2),
                )

            pen = QPen(colors[i % len(colors)])
            pen.setWidth(0.1)

            painter.setPen(pen)
            painter.drawPolyline(point_list)

        for i in range(len(self.points_list) - 1):
            p1 = self.image_to_view_factor * self.points_list[i][-1]
            p2 = self.image_to_view_factor * self.points_list[i + 1][0]

            pen = QPen(colors[i % len(colors)])
            pen.setWidth(0.1)

            painter.setPen(pen)
            painter.drawLine(QPointF(p1[0], p1[1]), QPointF(p2[0], p2[1]))

    def draw_points(self, painter: QPainter, points: np.array):
        """Draw a set of points as rectangles."""
        pen = QPen(QColor(255, 0.0, 255, int(255 * self.rendering_alpha)))
        painter.setPen(pen)

        points = points * self.image_to_view_factor
        size = 3
        rect_list = [
            QRectF(point[0] - 0.5 * size, point[1] - 0.5 * size, size, size) for point in points
        ]

        # cSpell:ignore rects
        painter.drawRects(rect_list)

    def draw_heatmap(self, painter: QPainter, heatmap: np.array, display_size: QSize):
        """Draws a heatmap as rectangles."""
        w, h = heatmap.shape
        x_scale = display_size.width() / float(w)
        y_scale = display_size.height() / float(h)

        for j in range(h):
            for i in range(w):
                color_index = min(16 * heatmap[j, i], 255)
                color = cv2.applyColorMap(
                    np.array([color_index], dtype=np.uint8), cv2.COLORMAP_JET
                ).flatten()
                color = QColor(color[2], color[1], color[0])
                color.setAlphaF(self.rendering_alpha)

                painter.setPen(QPen(color))
                painter.setBrush(QBrush(color))
                painter.drawRect(QRectF(i * x_scale, j * y_scale, x_scale, y_scale))

    def paint(self, painter: QPainter, option, widget):
        """Reimplemented method to perform all of the image related rendering operations."""
        self.widget_size = widget.size()

        if self.raw_pixmap.isNull():
            return

        if self.widget_size is None:
            return

        painter.setRenderHint(QPainter.Antialiasing)

        display_size = self.raw_pixmap.size()
        display_size.scale(self.widget_size, Qt.KeepAspectRatio)

        self.display_pixmap = self.raw_pixmap
        self.display_pixmap = self.raw_pixmap.scaled(
            display_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        display_size = self.display_pixmap.size()

        # This offset is needed to that the coordinate (0,0) is in the middle of the pixel rather than one of the corners
        offset = -0.5
        painter.drawPixmap(QPointF(offset, offset), self.display_pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        pen = QPen(QColor(255, 0, 255, 64))
        pen.setWidth(0.1)
        painter.setPen(pen)

        drawing_offset = 0.0

        # left frame
        painter.drawLine(
            QPointF(drawing_offset, drawing_offset),
            QPointF(drawing_offset, display_size.height() - 1 + drawing_offset),
        )

        # bottom frame
        painter.drawLine(
            QPointF(drawing_offset, display_size.height() - 1 + drawing_offset),
            QPointF(
                display_size.width() - 1 + drawing_offset,
                display_size.height() - 1 + drawing_offset,
            ),
        )

        # right frame
        painter.drawLine(
            QPointF(
                display_size.width() - 1 + drawing_offset,
                display_size.height() - 1 + drawing_offset,
            ),
            QPointF(display_size.width() - 1 + drawing_offset, drawing_offset),
        )

        # top frame
        painter.drawLine(
            QPointF(display_size.width() - 1 + drawing_offset, drawing_offset),
            QPointF(drawing_offset, drawing_offset),
        )

        self.width_image_to_view_factor = float(display_size.width()) / float(
            self.raw_pixmap.size().width()
        )
        self.height_image_to_view_factor = float(display_size.height()) / float(
            self.raw_pixmap.size().height()
        )
        self.image_to_view_factor = np.array(
            [self.width_image_to_view_factor, self.height_image_to_view_factor]
        )

        pen = QPen(QColor(255, 0, 0, 64))
        pen.setWidth(0.1)
        painter.setPen(pen)

        # left frame
        painter.drawLine(
            QPointF(0, 0),
            QPointF(0, self.height_image_to_view_factor * self.raw_pixmap.height() - 1),
        )

        # bottom frame
        painter.drawLine(
            QPointF(0, self.height_image_to_view_factor * self.raw_pixmap.height() - 1),
            QPointF(
                self.width_image_to_view_factor * self.raw_pixmap.width() - 1,
                self.height_image_to_view_factor * self.raw_pixmap.height() - 1,
            ),
        )

        # right frame
        painter.drawLine(
            QPointF(
                self.width_image_to_view_factor * self.raw_pixmap.width() - 1,
                self.height_image_to_view_factor * self.raw_pixmap.height() - 1,
            ),
            QPointF(self.width_image_to_view_factor * self.raw_pixmap.width() - 1, 0),
        )

        # top frame
        painter.drawLine(
            QPointF(self.width_image_to_view_factor * self.raw_pixmap.width() - 1, 0), QPointF(0, 0)
        )

        if self.is_draw_detection_points:
            self.draw_detection_points(painter)

        if self.is_draw_training_points:
            self.draw_points(painter, self.training_points)

        if self.is_draw_evaluation_points:
            self.draw_points(painter, self.evaluation_points)

        if self.is_draw_training_heatmap and self.training_heatmap is not None:
            self.draw_heatmap(painter, self.training_heatmap, display_size)

        if self.is_draw_evaluation_heatmap and self.evaluation_heatmap is not None:
            self.draw_heatmap(painter, self.evaluation_heatmap, display_size)

    def boundingRect(self):
        """Return the size of the Widget to let other widgets  adjust correctly."""
        if self.display_pixmap.size().width() == 0:
            return QRectF(0, 0, 500, 500)

        return QRectF(0, 0, self.display_pixmap.size().width(), self.display_pixmap.size().height())
