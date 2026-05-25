#!/usr/bin/env python3
"""Lesson 5 — Textured quad. Concepts: qt_cpp/opengl_tutorial/05_texture_quad/lesson.md"""

import math
import os
import sys

import numpy as np

_TUTORIAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM = os.path.dirname(_TUTORIAL)
sys.path[:0] = [_SIM, _TUTORIAL]

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_RGBA,
    GL_TEXTURE0,
    GL_TRIANGLES,
    GL_UNSIGNED_BYTE,
    GL_UNSIGNED_SHORT,
    glClear,
    glClearColor,
    glDrawElements,
    glViewport,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from helper_py import (
    ArrayBuffer,
    ElementArrayBuffer,
    GL_FRAGMENT_SHADER,
    GL_STATIC_DRAW,
    GL_VERTEX_SHADER,
    Program,
    Texture2D,
)
from _common import run_lesson

VERTEX_SHADER = """
#version 120
attribute vec2 coord;
attribute vec2 tex_coord;
uniform mat3 mat;
varying vec2 v_tex_coord;
void main(void) {
  vec3 p = mat * vec3(coord, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
  v_tex_coord = tex_coord;
}
"""

FRAGMENT_SHADER = """
#version 120
uniform sampler2D texture;
varying vec2 v_tex_coord;
void main(void) {
  gl_FragColor = texture2D(texture, v_tex_coord);
}
"""

TEXTURE_NAME = "tkmce.jpeg"


def rotation_matrix_2d(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    mat = np.eye(3, dtype=np.float32)
    mat[0, 0] = c
    mat[0, 1] = -s
    mat[1, 0] = s
    mat[1, 1] = c
    return mat


def make_checkerboard(size: int = 128, cells: int = 8) -> tuple[int, int, bytes]:
    cell = size // cells
    data = np.zeros((size, size, 4), dtype=np.uint8)
    for y in range(size):
        for x in range(size):
            dark = ((x // cell) + (y // cell)) % 2 == 0
            data[y, x] = [40, 40, 50, 255] if dark else [220, 220, 230, 255]
    return size, size, data.tobytes()


def load_tkmce_or_checkerboard() -> tuple[int, int, bytes]:
    search_dirs = [
        os.path.dirname(os.path.abspath(__file__)),
        _TUTORIAL,
        os.path.join(_TUTORIAL, "05_texture_quad"),
        os.path.normpath(
            os.path.join(_SIM, "..", "qt_cpp", "opengl_tutorial", "05_texture_quad")
        ),
    ]
    for directory in search_dirs:
        path = os.path.join(directory, TEXTURE_NAME)
        if not os.path.isfile(path):
            continue
        image = QImage(path)
        if image.isNull():
            continue
        image = image.convertToFormat(QImage.Format.Format_RGBA8888)
        ptr = image.constBits()
        return image.width(), image.height(), bytes(ptr)
    print(f"Texture: {TEXTURE_NAME} not found, using checkerboard")
    return make_checkerboard()


class TextureQuadWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self._angle = 0.0

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        corners = [[-0.5, -0.4], [0.5, -0.4], [0.5, 0.4], [-0.5, 0.4]]
        tex_coords = [[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]]
        indices = [0, 1, 2, 0, 2, 3]
        self._vertex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=corners, parent=self)
        self._tex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=tex_coords, parent=self)
        self._index_buffer = ElementArrayBuffer(GL_STATIC_DRAW, data=indices, parent=self)
        self._program = Program(
            [(GL_VERTEX_SHADER, VERTEX_SHADER), (GL_FRAGMENT_SHADER, FRAGMENT_SHADER)],
            parent=self,
        )
        w, h, pixels = load_tkmce_or_checkerboard()
        self._texture = Texture2D(
            GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, w, h, parent=self, data=pixels
        )

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        mat = rotation_matrix_2d(self._angle)
        with Program.Use(self._program, ["coord", "tex_coord"]):
            self._program.set_attribute("coord", self._vertex_buffer, components=2)
            self._program.set_attribute("tex_coord", self._tex_buffer, components=2)
            self._program.set_uniform("mat", mat)
            self._texture.bind(GL_TEXTURE0)
            self._program.set_uniform("texture", 0)
            self._index_buffer.bind()
            glDrawElements(
                GL_TRIANGLES, self._index_buffer.size(), GL_UNSIGNED_SHORT, None
            )
            self._index_buffer.unbind()
            self._texture.unbind(GL_TEXTURE0)

    def keyPressEvent(self, event):
        step = 0.08
        if event.key() == Qt.Key_A:
            self._angle += step
            self.update()
        elif event.key() == Qt.Key_D:
            self._angle -= step
            self.update()
        elif event.key() == Qt.Key_R:
            self._angle = 0.0
            self.update()
        else:
            super().keyPressEvent(event)


if __name__ == "__main__":
    run_lesson(
        TextureQuadWidget,
        "Python Lesson 5: Textured Quad (A/D rotate, R reset)",
    )
