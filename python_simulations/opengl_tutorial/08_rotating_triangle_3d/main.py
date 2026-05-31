#!/usr/bin/env python3
"""
Lesson 8 — 3D triangular pyramid with roll, pitch, and yaw (numpy).

Demonstrates the model–view–projection (MVP) pipeline:
  gl_Position = projection @ view @ model @ vec4(coord, 1)

Pyramid geometry: four triangles (12 vertices) in one glDrawArrays call.
Per-vertex colors (vert_color attribute) — same RGB repeated for each
triangle's three corners so each face is a solid color.

Controls (widget must have focus):
  Q/E — roll −/+    W/S — pitch +/−    A/D — yaw −/+    R — reset angles

Concepts: qt_cpp/opengl_tutorial/08_rotating_triangle_3d/lesson.md
"""

import math
import os
import sys

import numpy as np

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_BACK,
    GL_COLOR_BUFFER_BIT,
    GL_CULL_FACE,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_TRIANGLES,
    glClear,
    glClearColor,
    glCullFace,
    glDrawArrays,
    glEnable,
    glViewport,
)
from PySide6.QtCore import Qt
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_FRAGMENT_SHADER, GL_STATIC_DRAW, GL_VERTEX_SHADER, Program
from _common import run_lesson

VERTEX_SHADER_SOURCE = """
#version 120
attribute vec3 coord;
attribute vec3 vert_color;
uniform mat4 mvp;
varying vec3 frag_color;
void main(void) {
  gl_Position = mvp * vec4(coord, 1.0);
  frag_color = vert_color;
}
"""

FRAGMENT_SHADER_SOURCE = """
#version 120
varying vec3 frag_color;
void main(void) {
  gl_FragColor = vec4(frag_color, 1.0);
}
"""


def perspective(fov_y_deg: float, aspect: float, z_near: float, z_far: float) -> np.ndarray:
    """OpenGL-style perspective matrix (column vectors; GPU divides by w)."""
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
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
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
        glEnable(GL_DEPTH_TEST)  # nearer fragments win when triangles overlap
        glEnable(GL_CULL_FACE)   # hide back-facing triangles as the pyramid spins
        glCullFace(GL_BACK)

        # Triangular pyramid: four triangles (12 vertices), CCW when viewed from outside.
        apex = [0.0, 0.0, 0.55]
        b0 = [-0.55, -0.35, -0.35]
        b1 = [0.55, -0.35, -0.35]
        b2 = [0.0, 0.45, -0.35]

        vertices = [
            b0, b1, b2,       # base
            apex, b1, b0,     # side
            apex, b2, b1,     # side
            apex, b0, b2,     # side
        ]
        self._vbo = ArrayBuffer(GL_STATIC_DRAW, data=vertices, parent=self)

        blue = [0.0, 0.0, 1.0]
        red = [1.0, 0.0, 0.0]
        green = [0.0, 1.0, 0.0]
        yellow = [1.0, 0.85, 0.25]
        colors = [
            blue, blue, blue,
            red, red, red,
            green, green, green,
            yellow, yellow, yellow,
        ]
        self._color_buffer = ArrayBuffer(GL_STATIC_DRAW, data=colors, parent=self)

        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER_SOURCE), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER_SOURCE)],
            parent=self,
        )

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        self._aspect = width / height if height > 0 else 1.0

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        projection = perspective(45.0, self._aspect, 0.1, 10.0)
        view = np.eye(4, dtype=np.float32)
        view[2, 3] = -2.8  # view: push world back along −Z
        model = model_matrix(self._roll, self._pitch, self._yaw)
        mvp = projection @ view @ model  # clip = P @ V @ M @ v

        with Program.Use(self._program, ["coord", "vert_color"]):
            self._program.set_attribute("coord", self._vbo, components=3)
            self._program.set_attribute("vert_color", self._color_buffer, components=3)
            self._program.set_uniform("mvp", mvp)
            glDrawArrays(GL_TRIANGLES, 0, self._vbo.size())  # one draw for whole pyramid

    def keyPressEvent(self, event):
        step = 0.08  # radians per key press
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
        "Python Lesson 8: 3D Pyramid (Q/E roll, W/S pitch, A/D yaw, R reset)",
        depth_bits=24,
    )
