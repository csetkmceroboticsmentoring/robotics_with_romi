#!/usr/bin/env python3
"""Lesson 2 — Per-vertex colors. Concepts: qt_cpp/opengl_tutorial/02_vertex_colors/lesson.md"""

import os
import sys

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_TRIANGLES, glClear, glClearColor, glDrawArrays
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_FRAGMENT_SHADER, GL_STATIC_DRAW, GL_VERTEX_SHADER, Program
from _common import run_lesson

VERTEX_SHADER = """
#version 120
attribute vec2 coord;
attribute vec3 vert_color;
varying vec3 frag_color;
void main(void) {
  gl_Position = vec4(coord, 0.0, 1.0);
  frag_color = vert_color;
}
"""

FRAGMENT_SHADER = """
#version 120
varying vec3 frag_color;
void main(void) {
  gl_FragColor = vec4(frag_color, 1.0);
}
"""


class VertexColorsWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        positions = [
            [0.0, 0.45], [-0.45, -0.35], [0.45, -0.35],
            [-0.55, 0.1], [-0.85, -0.45], [-0.25, -0.45],
            [0.55, 0.1], [0.25, -0.45], [0.85, -0.45],
        ]
        colors = [
            [1.0, 0.2, 0.2], [1.0, 0.2, 0.2], [1.0, 0.2, 0.2],
            [0.2, 1.0, 0.3], [0.2, 1.0, 0.3], [0.2, 1.0, 0.3],
            [0.3, 0.5, 1.0], [0.3, 0.5, 1.0], [0.3, 0.5, 1.0],
        ]
        self._pos_buffer = ArrayBuffer(GL_STATIC_DRAW, data=positions, parent=self)
        self._color_buffer = ArrayBuffer(GL_STATIC_DRAW, data=colors, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )

    def resizeGL(self, width, height):
        from OpenGL.GL import glViewport

        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        with Program.Use(self._program, ["coord", "vert_color"]):
            self._program.set_attribute("coord", self._pos_buffer, components=2)
            self._program.set_attribute("vert_color", self._color_buffer, components=3)
            glDrawArrays(GL_TRIANGLES, 0, self._pos_buffer.size())


if __name__ == "__main__":
    run_lesson(VertexColorsWidget, "Python Lesson 2: Vertex Colors")
