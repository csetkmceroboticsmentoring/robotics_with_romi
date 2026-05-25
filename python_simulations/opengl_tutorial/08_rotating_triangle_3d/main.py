#!/usr/bin/env python3
"""Lesson 8 — 3D triangle MVP. Concepts: qt_cpp/opengl_tutorial/08_rotating_triangle_3d/lesson.md"""

import math
import os
import sys

import numpy as np

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_TRIANGLES,
    glClear,
    glClearColor,
    glDrawArrays,
    glEnable,
    glViewport,
)
from PySide6.QtCore import Qt
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_FRAGMENT_SHADER, GL_STATIC_DRAW, GL_VERTEX_SHADER, Program
from _common import run_lesson

VERTEX_SHADER = """
#version 120
attribute vec3 coord;
uniform mat4 mvp;
void main(void) {
  gl_Position = mvp * vec4(coord, 1.0);
}
"""

FRAGMENT_SHADER = """
#version 120
uniform vec3 color;
void main(void) {
  gl_FragColor = vec4(color, 1.0);
}
"""


def perspective(fov_y_deg: float, aspect: float, z_near: float, z_far: float) -> np.ndarray:
    f = 1.0 / math.tan(fov_y_deg * 0.5 * math.pi / 180.0)
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (z_far + z_near) / (z_near - z_far)
    m[2, 3] = (2.0 * z_far * z_near) / (z_near - z_far)
    m[3, 2] = -1.0
    return m


def rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float32
    )


def rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float32
    )


def rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32
    )


def rotation_from_roll_pitch_yaw(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def model_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    model = np.eye(4, dtype=np.float32)
    model[:3, :3] = rotation_from_roll_pitch_yaw(roll, pitch, yaw)
    return model


class RotatingTriangle3DWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self._aspect = 1.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glEnable(GL_DEPTH_TEST)
        vertices = [[0.0, 0.6, 0.0], [-0.6, -0.4, 0.0], [0.6, -0.4, 0.0]]
        self._vbo = ArrayBuffer(GL_STATIC_DRAW, data=vertices, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        self._aspect = width / height if height > 0 else 1.0

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        projection = perspective(45.0, self._aspect, 0.1, 10.0)
        view = np.eye(4, dtype=np.float32)
        view[2, 3] = -2.8
        model = model_matrix(self._roll, self._pitch, self._yaw)
        mvp = projection @ view @ model
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._vbo, components=3)
            self._program.set_uniform("mvp", mvp)
            self._program.set_uniform("color", 0.35, 0.75, 1.0)
            glDrawArrays(GL_TRIANGLES, 0, self._vbo.size())

    def keyPressEvent(self, event):
        step = 0.08
        if event.key() == Qt.Key_Q:
            self._roll -= step
        elif event.key() == Qt.Key_E:
            self._roll += step
        elif event.key() == Qt.Key_W:
            self._pitch += step
        elif event.key() == Qt.Key_S:
            self._pitch -= step
        elif event.key() == Qt.Key_A:
            self._yaw -= step
        elif event.key() == Qt.Key_D:
            self._yaw += step
        elif event.key() == Qt.Key_R:
            self._roll = self._pitch = self._yaw = 0.0
        else:
            super().keyPressEvent(event)
            return
        self.update()


if __name__ == "__main__":
    run_lesson(
        RotatingTriangle3DWidget,
        "Python Lesson 8: 3D Triangle (Q/E roll, W/S pitch, A/D yaw, R reset)",
        depth_bits=24,
    )
