#!/usr/bin/env python3
"""Lesson 1 — First triangle (NDC). Concepts: qt_cpp/opengl_tutorial/01_hello_triangle/lesson.md"""

import os
import sys

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_TRIANGLES,
    glClear,
    glClearColor,
    glDrawArrays,
    glViewport,
)
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import ArrayBuffer, GL_FRAGMENT_SHADER, GL_STATIC_DRAW, GL_VERTEX_SHADER, Program
from _common import run_lesson

VERTEX_SHADER = """
#version 120
attribute vec2 coord;
void main(void) {
  gl_Position = vec4(coord, 0.0, 1.0);
}
"""

FRAGMENT_SHADER = """
#version 120
uniform vec3 color;
void main(void) {
  gl_FragColor = vec4(color, 1.0);
}
"""


class HelloTriangleWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        vertices = [[0.0, 0.5], [-0.5, -0.5], [0.5, -0.5]]
        self._vbo = ArrayBuffer(GL_STATIC_DRAW, data=vertices, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        with Program.Use(self._program, ["coord"]):
            self._program.set_attribute("coord", self._vbo, components=2)
            self._program.set_uniform("color", 1.0, 0.0, 0.0)
            glDrawArrays(GL_TRIANGLES, 0, self._vbo.size())


if __name__ == "__main__":
    run_lesson(HelloTriangleWidget, "Python Lesson 1: Hello Triangle")
