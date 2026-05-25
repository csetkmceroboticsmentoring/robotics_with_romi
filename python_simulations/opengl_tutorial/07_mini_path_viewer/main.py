#!/usr/bin/env python3
"""Lesson 7 — Mini path viewer. Concepts: qt_cpp/opengl_tutorial/07_mini_path_viewer/lesson.md"""

import math
import os
import sys

import numpy as np

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_LINES,
    GL_LINE_STRIP,
    GL_TRIANGLES,
    glClear,
    glClearColor,
    glDrawArrays,
    glViewport,
)
from PySide6.QtCore import QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import (
    ArrayBuffer,
    GL_DYNAMIC_DRAW,
    GL_FRAGMENT_SHADER,
    GL_STATIC_DRAW,
    GL_VERTEX_SHADER,
    Program,
)
from _common import run_lesson

VERTEX_SHADER = """
#version 120
attribute vec2 coord;
uniform mat3 mat;
void main(void) {
  vec3 p = mat * vec3(coord, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
}
"""

FRAGMENT_SHADER = """
#version 120
uniform vec3 color;
void main(void) {
  gl_FragColor = vec4(color, 1.0);
}
"""


def pose_matrix(heading: float, pos: np.ndarray) -> np.ndarray:
    c, s = math.cos(heading), math.sin(heading)
    mat = np.eye(3, dtype=np.float32)
    mat[0, 0] = c
    mat[0, 1] = -s
    mat[1, 0] = s
    mat[1, 1] = c
    mat[0, 2] = pos[0]
    mat[1, 2] = pos[1]
    return mat


class MiniPathViewerWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._heading = 0.0
        self._pos = np.zeros(2, dtype=np.float32)
        self._traj_points = [[0.0, 0.0]]

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        self._init_grid()
        self._init_marker()
        self._traj_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=2000, parent=self)
        self._traj_buffer.load(self._traj_points)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._step_simulation)
        self._timer.start(40)

    def _init_grid(self):
        points = []
        num_lines = 11
        spacing = 2.0 / (num_lines - 1)
        for i in range(num_lines):
            x = -1.0 + i * spacing
            points.extend([[x, -1.0], [x, 1.0]])
        for i in range(num_lines):
            y = -1.0 + i * spacing
            points.extend([[-1.0, y], [1.0, y]])
        self._grid_buffer = ArrayBuffer(GL_STATIC_DRAW, data=points, parent=self)

    def _init_marker(self):
        s = 0.03
        body = [[s, 0.0], [-0.5 * s, 0.5 * s], [-0.5 * s, -0.5 * s]]
        self._marker_buffer = ArrayBuffer(GL_STATIC_DRAW, data=body, parent=self)

    def _step_simulation(self):
        self._heading += 0.04
        speed = 0.012
        self._pos[0] += speed * math.cos(self._heading)
        self._pos[1] += speed * math.sin(self._heading)
        self._traj_points.append([float(self._pos[0]), float(self._pos[1])])
        if len(self._traj_points) > 500:
            self._traj_points.pop(0)
        self._traj_buffer.load(self._traj_points)
        self.update()

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        screen_mat = np.eye(3, dtype=np.float32) * 0.95
        self._draw_lines(self._grid_buffer, 0.75, 0.75, 0.75, screen_mat)
        if self._traj_buffer.size() >= 2:
            self._draw_lines(self._traj_buffer, 0.1, 0.35, 0.9, screen_mat, GL_LINE_STRIP)
        self._draw_marker(screen_mat)

    def _draw_lines(self, buffer, r, g, b, mat, mode=GL_LINES):
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", buffer, components=2)
            self._program.set_uniform("mat", mat)
            self._program.set_uniform("color", r, g, b)
            glDrawArrays(mode, 0, buffer.size())

    def _draw_marker(self, screen_mat):
        marker_mat = screen_mat @ pose_matrix(self._heading, self._pos)
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._marker_buffer, components=2)
            self._program.set_uniform("mat", marker_mat)
            self._program.set_uniform("color", 0.9, 0.2, 0.15)
            glDrawArrays(GL_TRIANGLES, 0, 3)


if __name__ == "__main__":
    run_lesson(
        MiniPathViewerWidget,
        "Python Lesson 7: Mini Path Viewer",
        width=720,
        height=720,
    )
