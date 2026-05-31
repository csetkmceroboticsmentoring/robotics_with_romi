#!/usr/bin/env python3
"""
Lesson 4 — Indexed drawing with ElementArrayBuffer.

A quad uses 4 unique corners but 6 indices (two triangles: 0-1-2 and 0-2-3).
glDrawElements looks up vertices by index instead of drawing them in order.

Concepts: qt_cpp/opengl_tutorial/04_indexed_quad/lesson.md
"""

import os
import sys

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_TRIANGLES,
    GL_UNSIGNED_SHORT,
    glClear,
    glClearColor,
    glDrawElements,
    glViewport,
)
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import (
    ArrayBuffer,
    ElementArrayBuffer,
    GL_FRAGMENT_SHADER,
    GL_STATIC_DRAW,
    GL_VERTEX_SHADER,
    Program,
)
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


class IndexedQuadWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        corners = [[-0.4, -0.3], [0.4, -0.3], [0.4, 0.3], [-0.4, 0.3]]
        indices = [0, 1, 2, 0, 2, 3]
        self._vertex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=corners, parent=self)
        self._index_buffer = ElementArrayBuffer(GL_STATIC_DRAW, data=indices, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER_SOURCE), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER_SOURCE)],
            parent=self,
        )

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._vertex_buffer, components=2)
            self._program.set_uniform("color", 0.85, 0.45, 0.1)
            self._index_buffer.bind()
            # Indices start at offset 0 in the bound element array buffer.
            glDrawElements(
                GL_TRIANGLES, self._index_buffer.size(), GL_UNSIGNED_SHORT, None
            )
            self._index_buffer.unbind()


if __name__ == "__main__":
    run_lesson(IndexedQuadWidget, "Python Lesson 4: Indexed Quad")
