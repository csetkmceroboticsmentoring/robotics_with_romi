#!/usr/bin/env python3
"""
Lesson 6 — Dynamic vertex buffer (GL_DYNAMIC_DRAW).

Each frame rebuilds a spiral polyline on the CPU and uploads it with
ArrayBuffer.load(). Contrasts with GL_STATIC_DRAW in earlier lessons.

QTimer drives animation; paintGL always reflects the latest points.
Concepts: qt_cpp/opengl_tutorial/06_dynamic_line_strip/lesson.md
"""

import math
import os
import sys

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_LINE_STRIP,
    glClear,
    glClearColor,
    glDrawArrays,
    glViewport,
)
from PySide6.QtCore import QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_DYNAMIC_DRAW, GL_FRAGMENT_SHADER, GL_VERTEX_SHADER, Program
from _common import run_lesson

VERTEX_SHADER_SOURCE = """
#version 120
attribute vec2 coord;
void main(void) {
  gl_Position = vec4(coord, 0.0, 1.0);
}
"""

FRAGMENT_SHADER_SOURCE = """
#version 120
uniform vec3 color;
void main(void) {
  gl_FragColor = vec4(color, 1.0);
}
"""


class DynamicLineWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._t = 0.0  # phase angle for spiral animation

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        # Pre-allocate GPU capacity; load() fills actual vertex count each frame.
        self._line_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=4096, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER_SOURCE), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER_SOURCE)],
            parent=self,
        )
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(50)

    def _on_tick(self):
        self._t += 0.05
        self.update()

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        points = [[0.0, 0.0]]
        segments = 80
        for i in range(1, segments + 1):
            a = (i / segments) * 4.0 * math.pi + self._t
            r = 0.35 * (i / segments)
            points.append([r * math.cos(a), r * math.sin(a)])
        self._line_buffer.load(points)
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._line_buffer, components=2)
            self._program.set_uniform("color", 0.9, 0.15, 0.2)
            glDrawArrays(GL_LINE_STRIP, 0, self._line_buffer.size())


if __name__ == "__main__":
    run_lesson(DynamicLineWidget, "Python Lesson 6: Dynamic Line Strip")
