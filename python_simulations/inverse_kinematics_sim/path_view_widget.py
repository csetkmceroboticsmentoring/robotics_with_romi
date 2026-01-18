"""
OpenGL widget for visualizing inverse kinematics simulation with waypoint following.
"""

import sys
import os
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QSurfaceFormat, QKeyEvent, QMouseEvent, QImage
from OpenGL.GL import *

# Add parent directory to path for helper_py
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from helper_py import Program, ArrayBuffer, ElementArrayBuffer, Texture2D
from helper_py import GL_STATIC_DRAW, GL_DYNAMIC_DRAW
from helper_py import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER
from helper_py import GL_RGBA, GL_UNSIGNED_BYTE

from motion_control import MotionController


# Shader sources
vertex_shader = """
#version 120
attribute vec2 coord;
uniform mat3 mat;
void main(void) {
    vec3 new_pos = mat * vec3(coord.xy, 1.0);
    gl_Position = vec4(new_pos.xy, 0.0, 1.0);
}
"""

fragment_shader = """
#version 120
uniform vec3 color;
void main(void) {
    gl_FragColor = vec4(color, 1.0);
}
"""

texture_vertex_shader = """
#version 120
attribute vec2 coord;
attribute vec2 tex_coord;
uniform mat3 mat;
varying vec2 v_tex_coord;
void main(void) {
    vec3 new_pos = mat * vec3(coord.xy, 1.0);
    gl_Position = vec4(new_pos.xy, 0.0, 1.0);
    v_tex_coord = tex_coord;
}
"""

texture_fragment_shader = """
#version 120
uniform sampler2D texture;
varying vec2 v_tex_coord;
void main(void) {
    gl_FragColor = texture2D(texture, v_tex_coord);
}
"""


class PathViewWidget(QOpenGLWidget):
    """
    OpenGL widget for inverse kinematics simulation.
    Shows robot, trajectory, and waypoints.
    """
    
    newWayPoint = Signal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Set OpenGL format
        fmt = QSurfaceFormat()
        fmt.setVersion(4, 1)
        fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
        self.setFormat(fmt)
        
        # Motion controller
        self.controller = MotionController()
        
        # OpenGL resources
        self.bg_buffer = None
        self.program = None
        self.robot_buffer = None
        self.robot_tex_buffer = None
        self.robot_index_buffer = None
        self.robot_texture = None
        self.texture_program = None
        self.traj_buffer = None
        self.wp_buffer = None
        
        # Scale factor for view
        self.scale_factor = 1.0
        
        # Robot pose matrix
        self.matrix = np.eye(3, dtype=np.float32)
        
        # Waypoint list for rendering
        self.waypoints = []
        
        # Timer for simulation updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_simulation)
        self.timer.setInterval(25)  # 40 Hz update rate
        
    def initializeGL(self):
        """Initialize OpenGL resources."""
        # Set clear color
        glClearColor(1.0, 1.0, 1.0, 1.0)
        
        # Enable texturing
        glEnable(GL_TEXTURE_2D)
        
        # Enable blending for transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # Initialize resources
        self.init_background()
        self.init_robot_coords()
        self.init_shaders()
        self.load_robot_texture()
        
        # Create trajectory and waypoint buffers
        self.traj_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=100000, parent=self)
        self.wp_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=100000, parent=self)
        
        # Start simulation timer
        self.timer.start()
        
    def init_background(self):
        """Create background grid."""
        # 11x11 grid spanning -1 to +1
        points = []
        
        num_lines = 11
        spacing = 2.0 / (num_lines - 1)
        
        # Vertical lines
        for i in range(num_lines):
            x = -1.0 + i * spacing
            points.append([x, -1.0])
            points.append([x, 1.0])
        
        # Horizontal lines
        for i in range(num_lines):
            y = -1.0 + i * spacing
            points.append([-1.0, y])
            points.append([1.0, y])
        
        self.bg_buffer = ArrayBuffer(GL_STATIC_DRAW, data=points, parent=self)
        
    def init_robot_coords(self):
        """Create robot quad for texture mapping."""
        robot_size = 0.15  # meters
        
        # Quad vertices
        robot_points = [
            [-robot_size/2, -robot_size/2],  # Bottom-left
            [robot_size/2, -robot_size/2],   # Bottom-right
            [robot_size/2, robot_size/2],    # Top-right
            [-robot_size/2, robot_size/2]    # Top-left
        ]
        
        # Texture coordinates
        tex_coords = [
            [0.0, 1.0],  # Bottom-left
            [1.0, 1.0],  # Bottom-right
            [1.0, 0.0],  # Top-right
            [0.0, 0.0]   # Top-left
        ]
        
        # Triangle indices
        indices = [0, 1, 2, 0, 2, 3]
        
        self.robot_buffer = ArrayBuffer(GL_STATIC_DRAW, data=robot_points, parent=self)
        self.robot_tex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=tex_coords, parent=self)
        self.robot_index_buffer = ElementArrayBuffer(GL_STATIC_DRAW, data=indices, parent=self)
        
    def init_shaders(self):
        """Initialize shader programs."""
        # Basic shader program for lines
        self.program = Program([
            (GL_VERTEX_SHADER, vertex_shader),
            (GL_FRAGMENT_SHADER, fragment_shader)
        ], parent=self)
        
        # Texture shader program for robot
        self.texture_program = Program([
            (GL_VERTEX_SHADER, texture_vertex_shader),
            (GL_FRAGMENT_SHADER, texture_fragment_shader)
        ], parent=self)
        
    def load_robot_texture(self):
        """Load robot texture from file."""
        texture_path = os.path.join(os.path.dirname(__file__), 'romi_small.png')
        
        if not os.path.exists(texture_path):
            print(f"Warning: Robot texture not found at {texture_path}")
            return
        
        print(f"Loading robot texture from {texture_path}")
        image = QImage(texture_path)
        
        if image.isNull():
            print(f"Failed to load robot texture")
            return
        
        # Convert to RGBA format
        image = image.convertToFormat(QImage.Format.Format_RGBA8888)
        image = image.mirrored(False, True)  # Flip vertically for OpenGL
        
        # Get image data
        bits = image.constBits()
        data = np.frombuffer(bits, dtype=np.uint8).reshape((image.height(), image.width(), 4))
        
        print(f"Loaded image: {image.width()}x{image.height()}")
        
        self.robot_texture = Texture2D(
            GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE,
            image.width(), image.height(),
            self, data.tobytes()
        )
        
        print("Created texture successfully")
        
    def resizeGL(self, width, height):
        """Handle window resize."""
        glViewport(0, 0, width, height)
        
    def paintGL(self):
        """Render the scene."""
        self.draw_background()
        self.draw_robot()
        self.draw_waypoints()
        self.draw_trajectory()
        
    def draw_background(self):
        """Draw background grid."""
        glClear(GL_COLOR_BUFFER_BIT)
        
        # Draw grid
        with Program.Use(self.program, ["coord"]):
            self.program.set_attribute("coord", self.bg_buffer, components=2)
            self.program.set_uniform("color", 0.3, 0.3, 0.3)  # Dark gray
            
            # Create scale matrix
            mat = np.eye(3, dtype=np.float32) * self.scale_factor
            mat[2, 2] = 1.0
            self.program.set_uniform("mat", mat)
            
            glDrawArrays(GL_LINES, 0, self.bg_buffer.size())
            
    def draw_trajectory(self):
        """Draw robot trajectory."""
        if len(self.controller.trajectory) > 1:
            self.traj_buffer.load(self.controller.trajectory)
            
            with Program.Use(self.program, ["coord"]):
                self.program.set_attribute("coord", self.traj_buffer, components=2)
                self.program.set_uniform("color", 1.0, 0.0, 0.0)  # Red
                mat = np.eye(3, dtype=np.float32) * self.scale_factor
                mat[2, 2] = 1.0
                self.program.set_uniform("mat", mat)
                glDrawArrays(GL_LINE_STRIP, 0, len(self.controller.trajectory))
                
    def draw_waypoints(self):
        """Draw waypoints."""
        if len(self.waypoints) > 0:
            with Program.Use(self.program, ["coord"]):
                self.program.set_attribute("coord", self.wp_buffer, components=2)
                self.program.set_uniform("color", 0.0, 0.0, 1.0)  # Blue
                mat = np.eye(3, dtype=np.float32) * self.scale_factor
                mat[2, 2] = 1.0
                self.program.set_uniform("mat", mat)
                glDrawArrays(GL_LINE_STRIP, 0, self.wp_buffer.size())
                
    def draw_robot(self):
        """Draw robot at current position."""
        if not self.robot_texture:
            return
        
        # Update robot pose matrix
        c = np.cos(self.controller.heading)
        s = np.sin(self.controller.heading)
        self.matrix = np.array([
            [c, -s, self.controller.pos[0]],
            [s, c, self.controller.pos[1]],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        # Draw robot
        with Program.Use(self.texture_program, ["coord", "tex_coord"]):
            self.texture_program.set_attribute("coord", self.robot_buffer, components=2)
            self.texture_program.set_attribute("tex_coord", self.robot_tex_buffer, components=2)
            
            mat = self.matrix.copy()
            mat[:2, :] *= self.scale_factor
            self.texture_program.set_uniform("mat", mat)
            
            # Bind texture
            self.robot_texture.bind(GL_TEXTURE0)
            self.texture_program.set_uniform("texture", 0)
            
            # Draw robot
            self.robot_index_buffer.bind()
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, None)
            self.robot_index_buffer.unbind()
            
            self.robot_texture.unbind(GL_TEXTURE0)
            
    def update_simulation(self):
        """Update simulation state (called every 25ms)."""
        # Update motion controller
        x, y, heading = self.controller.update(dt=0.025)
        
        # Trigger repaint
        self.update()
        
    def keyPressEvent(self, event: QKeyEvent):
        """Handle key press."""
        # Reset on 'R'
        if event.key() == Qt.Key.Key_R:
            self.reset_simulation()
            
        # Zoom controls
        key = event.key()
        if key == Qt.Key.Key_PageUp or key == Qt.Key.Key_Up:
            self.scale_factor += 0.25
            self.update()
        elif key == Qt.Key.Key_PageDown or key == Qt.Key.Key_Down:
            self.scale_factor -= 0.25
            self.update()
            
    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse press to add waypoints."""
        if event.button() == Qt.MouseButton.LeftButton:
            # Convert screen coordinates to world coordinates
            p_x = event.pos().x() / (self.width() / 2.0) - 1.0
            p_y = 1.0 - event.pos().y() / (self.height() / 2.0)
            p_x /= self.scale_factor
            p_y /= self.scale_factor
            
            # Add waypoint to controller
            self.controller.add_waypoint(p_x, p_y)
            
            # Add to rendering list
            if len(self.waypoints) == 0:
                # First waypoint - add line from current position
                self.waypoints.append(self.controller.pos.tolist())
            self.waypoints.append([p_x, p_y])
            
            # Update buffer
            self.wp_buffer.load(self.waypoints)
            
            # Emit signal
            self.newWayPoint.emit(p_x, p_y)
            
            self.update()
            
    def clear(self):
        """Clear trajectory and waypoints."""
        self.controller.reset()
        self.waypoints.clear()
        if self.traj_buffer:
            self.traj_buffer.clear()
        if self.wp_buffer:
            self.wp_buffer.clear()
        self.update()
        
    def reset_simulation(self):
        """Reset simulation to initial state."""
        self.clear()

