"""
Example application demonstrating the helper_py OpenGL library.

This example creates a simple PySide6 OpenGL widget that draws multiple colored triangles
using the helper_py Buffer and Program classes.
"""

import sys
import os
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import Qt
from PySide6.QtGui import QSurfaceFormat
from OpenGL.GL import *

# Add parent directory to path to allow importing helper_py
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from helper_py import Buffer, Program, ArrayBuffer, GL_STATIC_DRAW
from helper_py import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER


class TriangleWidget(QOpenGLWidget):
    """OpenGL widget that draws multiple colored triangles."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._program = None
        self._vertex_buffer = None
        self._color_buffer = None
    
    def initializeGL(self):
        """Initialize OpenGL resources."""
        # Print OpenGL version info
        try:
            version = glGetString(GL_VERSION)
            glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION)
            if version and isinstance(version, bytes):
                print(f"OpenGL Version: {version.decode()}")
            elif version:
                print(f"OpenGL Version: {version}")
            if glsl_version and isinstance(glsl_version, bytes):
                print(f"GLSL Version: {glsl_version.decode()}")
            elif glsl_version:
                print(f"GLSL Version: {glsl_version}")
        except Exception as e:
            print(f"Could not get OpenGL version info: {e}")
        sys.stdout.flush()
        
        # Vertex shader source code (GLSL 120 / OpenGL 2.1)
        vertex_shader = """
        #version 120
        attribute vec2 position;
        attribute vec3 color;
        varying vec3 fragColor;
        
        void main() {
            gl_Position = vec4(position, 0.0, 1.0);
            fragColor = color;
        }
        """
        
        # Fragment shader source code (GLSL 120 / OpenGL 2.1)
        fragment_shader = """
        #version 120
        varying vec3 fragColor;
        
        void main() {
            gl_FragColor = vec4(fragColor, 1.0);
        }
        """
        
        # Create shader program
        print("Creating shader program...")
        sys.stdout.flush()
        try:
            self._program = Program([
                (GL_VERTEX_SHADER, vertex_shader),
                (GL_FRAGMENT_SHADER, fragment_shader)
            ], parent=self)
            print(f"Shader program created successfully, ID: {self._program.program_id()}")
            sys.stdout.flush()
        except Exception as e:
            print(f"ERROR creating shader program: {e}")
            sys.stderr.flush()
            return
        
        # Define triangle vertices and colors
        # Triangle 1: Red triangle at the top
        triangle1_vertices = [
            [0.0, 0.5],   # Top vertex
            [-0.5, -0.3], # Bottom left
            [0.5, -0.3]   # Bottom right
        ]
        triangle1_colors = [
            [1.0, 0.0, 0.0],  # Red
            [1.0, 0.0, 0.0],  # Red
            [1.0, 0.0, 0.0]   # Red
        ]
        
        # Triangle 2: Green triangle on the left
        triangle2_vertices = [
            [-0.6, 0.0],  # Top vertex
            [-0.9, -0.5], # Bottom left
            [-0.3, -0.5]  # Bottom right
        ]
        triangle2_colors = [
            [0.0, 1.0, 0.0],  # Green
            [0.0, 1.0, 0.0],  # Green
            [0.0, 1.0, 0.0]   # Green
        ]
        
        # Triangle 3: Blue triangle on the right
        triangle3_vertices = [
            [0.6, 0.0],   # Top vertex
            [0.3, -0.5],  # Bottom left
            [0.9, -0.5]   # Bottom right
        ]
        triangle3_colors = [
            [0.0, 0.0, 1.0],  # Blue
            [0.0, 0.0, 1.0],  # Blue
            [0.0, 0.0, 1.0]   # Blue
        ]
        
        # Triangle 4: Yellow triangle at the bottom
        triangle4_vertices = [
            [0.0, -0.6],  # Top vertex
            [-0.3, -0.9], # Bottom left
            [0.3, -0.9]   # Bottom right
        ]
        triangle4_colors = [
            [1.0, 1.0, 0.0],  # Yellow
            [1.0, 1.0, 0.0],  # Yellow
            [1.0, 1.0, 0.0]   # Yellow
        ]
        
        # Combine all triangles into separate lists
        all_vertices = []
        all_colors = []
        
        for vertices, colors in [
            (triangle1_vertices, triangle1_colors),
            (triangle2_vertices, triangle2_colors),
            (triangle3_vertices, triangle3_colors),
            (triangle4_vertices, triangle4_colors)
        ]:
            for vertex, color in zip(vertices, colors):
                all_vertices.append(vertex)
                all_colors.append(color)
        
        # Create vertex buffer with positions
        print(f"Creating vertex buffer with {len(all_vertices)} vertices")
        sys.stdout.flush()
        self._vertex_buffer = ArrayBuffer(
            GL_STATIC_DRAW,
            data=all_vertices,
            parent=self
        )
        print(f"Vertex buffer created, ID: {self._vertex_buffer.buffer_id()}, size: {self._vertex_buffer.size()}")
        sys.stdout.flush()
        
        # Create color buffer
        print(f"Creating color buffer with {len(all_colors)} colors")
        sys.stdout.flush()
        self._color_buffer = ArrayBuffer(
            GL_STATIC_DRAW,
            data=all_colors,
            parent=self
        )
        print(f"Color buffer created, ID: {self._color_buffer.buffer_id()}, size: {self._color_buffer.size()}")
        sys.stdout.flush()
        
        # Check attribute locations
        pos_loc = self._program.get_attribute_location("position")
        color_loc = self._program.get_attribute_location("color")
        print(f"Attribute locations - position: {pos_loc}, color: {color_loc}")
        sys.stdout.flush()
        
        # Set clear color to dark gray
        glClearColor(1.0, 1.0, 1.0, 1.0)
        print("initializeGL completed successfully")
        sys.stdout.flush()
    
    def resizeGL(self, width, height):
        """Handle widget resize."""
        glViewport(0, 0, width, height)
    
    def paintGL(self):
        """Render the scene."""
        # Clear the screen
        glClear(GL_COLOR_BUFFER_BIT)
        
        if not self._program or not self._vertex_buffer or not self._color_buffer:
            print("ERROR: Resources not initialized")
            sys.stderr.flush()
            return
        
        # Use the shader program with context manager for automatic cleanup
        print(f"paintGL: Using program {self._program.program_id()}")
        sys.stdout.flush()
        
        with Program.Use(self._program, ["position", "color"]):
            # Set up vertex attributes
            print("Setting vertex attributes...")
            sys.stdout.flush()
            self._program.set_attribute("position", self._vertex_buffer, components=2)
            self._program.set_attribute("color", self._color_buffer, components=3)
            
            # Draw all triangles (4 triangles * 3 vertices = 12 vertices)
            vertex_count = self._vertex_buffer.size()
            print(f"Drawing {vertex_count} vertices...")
            sys.stdout.flush()
            glDrawArrays(GL_TRIANGLES, 0, vertex_count)
            
            # Check for OpenGL errors
            error = glGetError()
            if error != GL_NO_ERROR:
                print(f"OpenGL Error: {error}")
                sys.stderr.flush()
            else:
                print("No OpenGL errors")
                sys.stdout.flush()


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("helper_py Example - Colored Triangles")
        self.setGeometry(100, 100, 800, 600)
        
        # Create and set the OpenGL widget
        self.gl_widget = TriangleWidget(self)
        self.setCentralWidget(self.gl_widget)


def main():
    """Main entry point."""
    # Set the default surface format before creating QApplication
    fmt = QSurfaceFormat()
    fmt.setVersion(4, 1)  # OpenGL 4.1
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)  # Compatibility profile for GLSL 120
    QSurfaceFormat.setDefaultFormat(fmt)
    
    app = QApplication(sys.argv)
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
