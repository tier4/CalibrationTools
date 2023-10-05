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

from collections import defaultdict
from typing import Dict
from typing import List
from typing import Optional
from typing import Set

from PySide2.QtCore import QXmlStreamReader
from PySide2.QtCore import Qt
from PySide2.QtGui import QPainter
from PySide2.QtSvg import QSvgRenderer
from PySide2.QtWidgets import QGraphicsScene
from PySide2.QtWidgets import QGraphicsView
from PySide2.QtWidgets import QWidget
import pydot
import transforms3d

# import debugpy
# debugpy.listen(5678)
# debugpy.wait_for_client()


class TfNode:
    def __init__(self, frame, transform):
        self.frame = frame
        self.transform = transform
        self.children = []

    def asDict(self):
        d = defaultdict(lambda: defaultdict(list))

        for child in self.children:
            d[self.frame][child.frame] = child.transform
            d.update(child.asDict())

        return d


class TfTree:
    def __init__(self, tf_dict):
        self.roots = []
        self.from_tf_dict(tf_dict)

    def from_tf_dict(self, tf_dicts):
        child_to_parent_dict = {}

        for parent, children in tf_dicts.items():
            for child in children.keys():
                child_to_parent_dict[child] = parent

        root_frames = []
        for child in child_to_parent_dict.keys():
            aux = child
            while aux in child_to_parent_dict:
                aux = child_to_parent_dict[aux]

            root_frames.append(aux)

        root_frames = list(set(root_frames))

        self.roots = [self.parseNode(root_frame, tf_dicts) for root_frame in root_frames]

    def parseNode(self, frame, tf_dicts):
        node = TfNode(frame, None)

        for child_frame, transform in tf_dicts[frame].items():
            if child_frame not in tf_dicts:
                node.children.append(TfNode(child_frame, transform))
            else:
                child = self.parseNode(child_frame, tf_dicts)
                child.transform = transform
                node.children.append(child)

        return node

    def getSlicedTree(
        self, current_node: TfNode, target_frames: List[str], current_frames: List[str] = []
    ):
        if len(current_node.children) == 0:
            return (
                (current_node, [current_node.frame])
                if current_node.frame in target_frames
                else (None, [])
            )

        sliced_children = []
        sliced_children_frames = []

        for children in current_node.children:
            sliced_node, sliced_frames = self.getSlicedTree(children, target_frames, current_frames)

            if len(sliced_frames) == len(target_frames):
                return sliced_node, sliced_frames
            elif sliced_node is not None:
                sliced_children.append(sliced_node)
                sliced_children_frames += sliced_frames

        sliced_node = TfNode(current_node.frame, current_node.transform)
        sliced_node.children = sliced_children

        if current_node.frame in target_frames:
            sliced_children_frames = sliced_children_frames + [current_node.frame]

        return (
            (sliced_node, sliced_children_frames)
            if len(sliced_children) > 0
            else (None, sliced_children_frames)
        )

    def getSlicesTrees(self, target_frames):
        sliced_trees = [self.getSlicedTree(root, target_frames)[0] for root in self.roots]

        return [sliced_tree for sliced_tree in sliced_trees if sliced_tree is not None]


class TfPlot(QWidget):
    def __init__(self):
        super(TfPlot, self).__init__()

        self.renderer = None
        self.setStyleSheet("background-color:white;")

    def setTfs(
        self,
        tfs_dict: Dict[Dict, str],
        changed_frames_dict: Optional[Dict[str, Set[str]]] = None,
        required_frames: Optional[List] = None,
    ):
        if required_frames:
            tree = TfTree(tfs_dict)
            sliced_nodes = tree.getSlicesTrees(required_frames)
            sliced_dict = {}
            [sliced_dict.update(sliced_node.asDict()) for sliced_node in sliced_nodes]
            tfs_dict = sliced_dict

        graph = pydot.Dot("my_graph", graph_type="digraph", bgcolor="white")

        parents_set = set(tfs_dict.keys())
        children_set = {
            child_frame for children_frames in tfs_dict.values() for child_frame in children_frames
        }

        nodes = list(parents_set.union(children_set))
        edges = [
            (parent_frame, child_frame)
            for parent_frame, children_frames in tfs_dict.items()
            for child_frame in children_frames
        ]

        # Add nodes
        # for node in nodes:
        #    graph.add_node(pydot.Node(node))

        # for parent, child in edges:
        #    graph.add_edge(pydot.Edge(parent, child))

        graph_list = []
        graph_list.append("digraph G {\n")

        for parent, child in edges:
            transform = tfs_dict[parent][child]
            translation = transform.translation
            quat = transform.rotation
            x, y, z = translation.x, translation.y, translation.z
            roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w, quat.x, quat.y, quat.z])
            color = (
                "red"
                if changed_frames_dict is not None
                and parent in changed_frames_dict
                and child in changed_frames_dict[parent]
                else "black"
            )
            graph_list.append(
                f'"{parent}" -> "{child}"[color={color} label=" x={x:.4f}\\ny={y:.4f}\\nz={z:.4f}\\nyaw={yaw:.4f}\\npitch={pitch:.4f}\\nroll={roll:.4f}\\n"];\n'
            )

        if changed_frames_dict is not None:
            changed_nodes = set()
            [
                changed_nodes.add(parent) or changed_nodes.add(child)
                for parent, children in changed_frames_dict.items()
                for child in children
            ]
            for node in nodes:
                color = "red" if node in changed_nodes else "black"
                graph_list.append(f'"{node}" [color="{color}"]')

        graph_list.append("}")
        graph_string = "".join(graph_list)
        # print(graph_string)
        graphs = pydot.graph_from_dot_data(graph_string)
        graph = graphs[0]

        # graph.write_png("frames.png")
        # graph.write_pdf("frames.pdf")

        imgdata = graph.create_svg()

        # imgdata = StringIO()
        # figure.savefig(imgdata, format='svg')
        # imgdata.seek(0)
        xmlreader = QXmlStreamReader(imgdata)
        self.renderer = QSvgRenderer(xmlreader)
        self.renderer.setAspectRatioMode(Qt.AspectRatioMode.KeepAspectRatio)
        # print(f"bits post savefig= {len(imgdata.getvalue())}", flush=True)

    def paintEvent(self, event):
        if self.renderer is None:
            return

        p = QPainter()
        p.begin(self)
        self.renderer.render(p)
        p.end()
        # print(self.geometry())
        # print(f"PAINTED", flush=True)


class TfView(QGraphicsView):
    def __init__(self):
        QGraphicsView.__init__(self)

        scene = QGraphicsScene(self)
        self.scene = scene

        self.plot = TfPlot()
        scene.addWidget(self.plot)

        self.setScene(scene)

        self.is_pan_active = False
        self.pan_start_x = None
        self.pan_start_y = None

        self.setMinimumWidth(400)

    def setTfs(
        self,
        tfs_dict,
        changed_frames_dict: Optional[Dict[str, Set[str]]] = None,
        required_frames: Optional[List] = None,
    ):
        self.plot.setTfs(tfs_dict, changed_frames_dict, required_frames)
        # Reset the view
        self.fitInView(0, 0, self.plot.width(), self.plot.height(), Qt.KeepAspectRatio)
        # print(f"=======Tf plot size: {self.plot.size()}", flush=True)

    def resizeEvent(self, event):
        print(
            f"PRE resizeEvent: event.size()={event.size()} event.oldSize()={event.oldSize()}",
            flush=True,
        )
        super().resizeEvent(event)
        print(
            f"POST resizeEvent: event.size()={event.size()} event.oldSize()={event.oldSize()}",
            flush=True,
        )

        # scaled_pix_size = self.pix.size()
        # scaled_pix_size.scale(self.data_renderer.widget_size, Qt.KeepAspectRatio)

        # import debugpy
        # debugpy.listen(5678)
        # debugpy.wait_for_client()

        for item in self.scene.items():
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

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.fitInView(0, 0, self.plot.width(), self.plot.height(), Qt.KeepAspectRatio)
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

        for item in self.scene.items():
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

        #####
        # targetScenePos = self.mapToScene(event.pos())
        # oldAnchor = self.transformationAnchor()
        # self.setTransformationAnchor(QGraphicsView.NoAnchor)

        # matrix = self.transform()
        # matrix.translate(targetScenePos.x(), targetScenePos.y()).scale(zoom_factor, zoom_factor).translate(-targetScenePos.x(), -targetScenePos.y())
        # self.setTransform(matrix)

        # self.setTransformationAnchor(oldAnchor)
        ####

        # Get the new position
        new_pos = self.mapToScene(event.pos())

        # Move scene to old position
        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())
