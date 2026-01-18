"""
Example application with file-based logging for debugging.
"""

import sys
import os
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import Qt
from PySide6.QtGui import QSurfaceFormat
from OpenGL.GL import *

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from helper_py import Buffer, Program, ArrayBuffer, GL_STATIC_DRAW
from helper_py import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER

# Open debug log file
debug_log = open("debug_log.txt", "w")

def log(msg):
    """Write to both console and file."""
    print(msg)
    debug_log.write(msg + "\n")
    debug_log.flush()
    sys.stdout.flush()


class TriangleWidget(QOpenGLWidget):
    """OpenGL widget that draws multiple colored triangles."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._program = None
        self._vertex_buffer = None
        self._color_buffer = None
        log("TriangleWidget created")
    
    def initializeGL(self):
        """Initialize OpenGL resources."""
        log("=== initializeGL START ===")
        
        # Print OpenGL version info
        try:
            version = glGetString(GL_VERSION)
            glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION)
            log(f"Raw version type: {type(version)}, value: {version}")
            if version and isinstance(version, bytes):
                log(f"OpenGL Version: {version.decode()}")
            elif version:
                log(f"OpenGL Version: {version}")
            if glsl_version and isinstance(glsl_version, bytes):
                log(f"GLSL Version: {glsl_version.decode()}")
            elif glsl_version:
                log(f"GLSL Version: {glsl_version}")
        except Exception as e:
            log(f"Could not get OpenGL version info: {e}")
            import traceback
            log(traceback.format_exc())
        
        # Vertex shader
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
        
        # Fragment shader
        fragment_shader = """
        #version 120
        varying vec3 fragColor;
        
        void main() {
            gl_FragColor = vec4(fragColor, 1.0);
        }
        """
        
        # Create shader program
        log("Creating shader program...")
        try:
            self._program = Program([
                (GL_VERTEX_SHADER, vertex_shader),
                (GL_FRAGMENT_SHADER, fragment_shader)
            ], parent=self)
            log(f"Shader program created, ID: {self._program.program_id()}")
        except Exception as e:
            log(f"ERROR creating shader program: {e}")
            import traceback
            log(traceback.format_exc())
            return
        
        # Triangle vertices
        all_vertices = [
            # Red triangle
            [0.0, 0.5], [-0.5, -0.3], [0.5, -0.3],
            # Green triangle
            [-0.6, 0.0], [-0.9, -0.5], [-0.3, -0.5],
            # Blue triangle
            [0.6, 0.0], [0.3, -0.5], [0.9, -0.5],
            # Yellow triangle
            [0.0, -0.6], [-0.3, -0.9], [0.3, -0.9]
        ]
        
        all_colors = [
            # Red
            [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0],
            # Green
            [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
            # Blue
            [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0],
            # Yellow
            [1.0, 1.0, 0.0], [1.0, 1.0, 0.0], [1.0, 1.0, 0.0]
        ]
        
        log(f"Creating buffers with {len(all_vertices)} vertices and {len(all_colors)} colors")
        
        # Create buffers
        try:
            log("Creating vertex buffer...")
            self._vertex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=all_vertices, parent=self)
            log(f"Vertex buffer created, ID: {self._vertex_buffer.buffer_id()}, size: {self._vertex_buffer.size()}")
            
            log("Creating color buffer...")
            self._color_buffer = ArrayBuffer(GL_STATIC_DRAW, data=all_colors, parent=self)
            log(f"Color buffer created, ID: {self._color_buffer.buffer_id()}, size: {self._color_buffer.size()}")
        except Exception as e:
            log(f"ERROR creating buffers: {e}")
            import traceback
            log(traceback.format_exc())
            return
        
        # Check attribute locations
        pos_loc = self._program.get_attribute_location("position")
        color_loc = self._program.get_attribute_location("color")
        log(f"Attribute locations - position: {pos_loc}, color: {color_loc}")
        
        # Set clear color
        self.glClearColor(1.0, 1.0, 1.0, 1.0)
        log("=== initializeGL COMPLETE ===")
    
    def resizeGL(self, width, height):
        """Handle widget resize."""
        glViewport(0, 0, width, height)
        log(f"Viewport resized to {width}x{height}")
    
    def paintGL(self):
        """Render the scene."""
        log("=== paintGL START ===")
        
        # Clear
        glClear(GL_COLOR_BUFFER_BIT)
        
        if not self._program or not self._vertex_buffer or not self._color_buffer:
            log("ERROR: Resources not initialized")
            return
        
        # Use shader
        log(f"Using program {self._program.program_id()}")
        
        try:
            with Program.Use(self._program, ["position", "color"]):
                log("Program bound, setting attributes...")
                
                # Set attributes
                self._program.set_attribute("position", self._vertex_buffer, components=2)
                self._program.set_attribute("color", self._color_buffer, components=3)
                log("Attributes set")
                
                # Draw
                vertex_count = self._vertex_buffer.size()
                log(f"Drawing {vertex_count} vertices...")
                glDrawArrays(GL_TRIANGLES, 0, vertex_count)
                
                # Check errors
                error = glGetError()
                if error != GL_NO_ERROR:
                    log(f"OpenGL Error: {error}")
                else:
                    log("Draw complete, no errors")
        except Exception as e:
            log(f"ERROR during rendering: {e}")
        
        log("=== paintGL END ===")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("helper_py Debug - Colored Triangles")
        self.setGeometry(100, 100, 800, 600)
        
        self.gl_widget = TriangleWidget(self)
        self.setCentralWidget(self.gl_widget)
        log("MainWindow created")


def main():
    log("=== APPLICATION START ===")
    
    # Set surface format
    fmt = QSurfaceFormat()
    fmt.setVersion(4, 1)
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
    QSurfaceFormat.setDefaultFormat(fmt)
    log("Surface format set to OpenGL 4.1 Compatibility Profile")
    
    app = QApplication(sys.argv)
    log("QApplication created")
    
    window = MainWindow()
    window.show()
    log("Window shown")
    
    log("Entering event loop...")
    result = app.exec()
    log(f"Event loop exited with code {result}")
    
    debug_log.close()
    sys.exit(result)


if __name__ == "__main__":
    main()

