"""
Shared setup for python_simulations/opengl_tutorial lessons.

- ensure_paths: import helper_py when running a lesson script directly.
- configure_gl_format: OpenGL 4.1 compatibility profile for GLSL #version 120.
- run_lesson: create QApplication, widget, and event loop.
"""

from __future__ import annotations

import os
import sys

from PySide6.QtGui import QSurfaceFormat
from PySide6.QtWidgets import QApplication

TUTORIAL_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_SIM_ROOT = os.path.dirname(TUTORIAL_DIR)


def ensure_paths() -> None:
    """Allow `from helper_py import ...` and `from _common import ...`."""
    for path in (PYTHON_SIM_ROOT, TUTORIAL_DIR):
        if path not in sys.path:
            sys.path.insert(0, path)


def configure_gl_format(*, depth_bits: int = 0) -> None:
    """Must run before QApplication when using a custom GL format (lesson 8: depth_bits=24)."""
    fmt = QSurfaceFormat()
    fmt.setRenderableType(QSurfaceFormat.OpenGL)
    fmt.setVersion(4, 1)
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
    if depth_bits > 0:
        fmt.setDepthBufferSize(depth_bits)
    QSurfaceFormat.setDefaultFormat(fmt)


def run_lesson(
    widget_class,
    title: str,
    width: int = 640,
    height: int = 480,
    *,
    depth_bits: int = 0,
) -> None:
    ensure_paths()
    configure_gl_format(depth_bits=depth_bits)
    app = QApplication(sys.argv)
    widget = widget_class()
    widget.setWindowTitle(title)
    widget.resize(width, height)
    widget.show()
    sys.exit(app.exec())
