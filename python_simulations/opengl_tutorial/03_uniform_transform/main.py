#!/usr/bin/env python3
"""Lesson 3 — uniform mat3 scale. Concepts: qt_cpp/opengl_tutorial/03_uniform_transform/lesson.md"""

import os
import sys

import numpy as np

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_LINE_STRIP,
    glClear,
    glClearColor,
    glDrawArrays,
)
from PySide6.QtCore import Qt
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_FRAGMENT_SHADER, GL_STATIC_DRAW, GL_VERTEX_SHADER, Program
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


class UniformTransformWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self._scale = 1.0

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        square = [
            [-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25], [-0.25, -0.25],
        ]
        self._line_buffer = ArrayBuffer(GL_STATIC_DRAW, data=square, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )

    def resizeGL(self, width, height):
        from OpenGL.GL import glViewport

        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        mat = np.eye(3, dtype=np.float32)
        mat[0, 0] = self._scale
        mat[1, 1] = self._scale
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._line_buffer, components=2)
            self._program.set_uniform("mat", mat)
            self._program.set_uniform("color", 0.2, 0.4, 0.85)
            glDrawArrays(GL_LINE_STRIP, 0, self._line_buffer.size())

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Plus, Qt.Key_Equal):
            self._scale = min(3.0, self._scale * 1.1)
            self.update()
        elif event.key() == Qt.Key_Minus:
            self._scale = max(0.2, self._scale / 1.1)
            self.update()
        else:
            super().keyPressEvent(event)


if __name__ == "__main__":
    run_lesson(
        UniformTransformWidget,
        "Python Lesson 3: Uniform Transform (+/- to scale)",
    )
