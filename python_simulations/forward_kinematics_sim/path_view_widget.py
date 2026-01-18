"""
OpenGL widget for visualizing robot forward kinematics simulation.
"""

import sys
import os
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QSurfaceFormat, QKeyEvent, QImage
from OpenGL.GL import *

# Add parent directory to path for helper_py
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from helper_py import Buffer, Program, ArrayBuffer, ElementArrayBuffer, Texture2D
from helper_py import GL_STATIC_DRAW, GL_DYNAMIC_DRAW
from helper_py import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER
from helper_py import GL_RGBA, GL_UNSIGNED_BYTE

# Import robot models
from robot_models import (
    WheelWithEncoder, 
    encoder_ticks_to_distance, 
    encoder_ticks_to_angle,
    DISTANCE_BETWEEN_WHEELS
)

# Import virtual controller
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'virtual_controller'))
from control_widget import ControlWidget


class PathViewWidget(QOpenGLWidget):
    """
    OpenGL widget that visualizes the robot simulation.
    Shows the robot, its trajectory, and a background grid.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Set OpenGL format
        fmt = QSurfaceFormat()
        fmt.setVersion(4, 1)
        fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
        self.setFormat(fmt)
        
        # Wheel simulators with PID parameters (from C++ version)
        self.left_wheel = WheelWithEncoder(kp=0.1613, ki=12.9032, kd=0.0)
        self.right_wheel = WheelWithEncoder(kp=0.1613, ki=12.9032, kd=0.0)
        
        # Robot state (odometry estimate)
        self.curr_heading = 0.0  # radians
        self.curr_pos = np.array([0.0, 0.0])  # [x, y] in meters
        
        # True robot state (ground truth)
        self.true_heading = 0.0
        self.true_pos = np.array([0.0, 0.0])
        
        # Trajectories
        self.traj_points = []
        self.true_traj_points = []
        
        # OpenGL resources
        self.bg_buffer = None
        self.program = None
        self.robot_buffer = None
        self.robot_tex_buffer = None
        self.robot_index_buffer = None
        self.robot_texture = None
        self.texture_program = None
        self.traj_buffer = None
        self.true_traj_buffer = None
        
        # Scale factor for view
        self.scale_factor = 1.0
        
        # Input state
        self.keys_pressed = set()
        
        # Create control widget as overlay (initially hidden)
        self.control_widget = ControlWidget(self)
        self.control_widget.setWindowFlags(Qt.WindowType.Widget)
        self.control_widget.hide()
        
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
        
        # Enable blending for transparency/fade effects
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # Initialize resources
        self.init_background()
        self.init_robot_coords()
        self.init_shaders()
        self.load_robot_texture()
        
        # Create trajectory buffers
        self.traj_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=100000, parent=self)
        self.true_traj_buffer = ArrayBuffer(GL_DYNAMIC_DRAW, max_items=100000, parent=self)
        
        # Start simulation timer
        self.timer.start()
        
        # Show control widget (since we're not supporting gamepad)
        self.position_control_widget()
        self.control_widget.show()
        
    def init_background(self):
        """Create background grid."""
        # Create a grid with vertical and horizontal lines
        # Grid spans from -1 to +1 in both directions
        # 11 lines in each direction (spacing of 0.2)
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
        robot_size = 0.17  # meters
        
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
        
        # Indices for two triangles
        indices = [0, 1, 2, 0, 2, 3]
        
        self.robot_buffer = ArrayBuffer(GL_STATIC_DRAW, data=robot_points, parent=self)
        self.robot_tex_buffer = ArrayBuffer(GL_STATIC_DRAW, data=tex_coords, parent=self)
        self.robot_index_buffer = ElementArrayBuffer(GL_STATIC_DRAW, data=indices, parent=self)
        
    def init_shaders(self):
        """Create shader programs."""
        # Basic shader for lines and shapes
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
        
        self.program = Program([
            (GL_VERTEX_SHADER, vertex_shader),
            (GL_FRAGMENT_SHADER, fragment_shader)
        ], parent=self)
        
        # Texture shader for robot image
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
        uniform float fade;
        uniform sampler2D texture;
        varying vec2 v_tex_coord;
        void main(void) {
            gl_FragColor = texture2D(texture, v_tex_coord) * fade;
        }
        """
        
        self.texture_program = Program([
            (GL_VERTEX_SHADER, texture_vertex_shader),
            (GL_FRAGMENT_SHADER, texture_fragment_shader)
        ], parent=self)
        
    def load_robot_texture(self):
        """Load robot texture image."""
        # Load from the same directory as this script
        texture_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "romi_small.png"
        )
        
        if not os.path.exists(texture_path):
            print(f"Warning: Could not find robot texture at {texture_path}")
            # Create a simple colored texture as fallback
            width, height = 64, 64
            data = np.ones((height, width, 4), dtype=np.uint8) * 255
            data[:, :, :3] = [100, 150, 200]  # Blue-ish color
            print("Using fallback blue texture")
        else:
            print(f"Loading robot texture from {texture_path}")
            # Load image using PySide6
            image = QImage(texture_path)
            if image.isNull():
                print(f"Warning: Failed to load image from {texture_path}, using fallback")
                # Use fallback
                width, height = 64, 64
                data = np.ones((height, width, 4), dtype=np.uint8) * 255
                data[:, :, :3] = [100, 150, 200]  # Blue-ish color
            else:
                # Convert to RGBA format
                image = image.convertToFormat(QImage.Format_RGBA8888)
                width = image.width()
                height = image.height()
                print(f"Loaded image: {width}x{height}")
                
                # Get pixel data
                ptr = image.constBits()
                data = np.array(ptr).reshape(height, width, 4)
        
        # Create texture
        self.robot_texture = Texture2D(
            GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE,
            width, height,
            parent=self,
            data=data.tobytes()
        )
        print(f"Created texture successfully")
        
    def resizeGL(self, width, height):
        """Handle window resize."""
        glViewport(0, 0, width, height)
        self.position_control_widget()
    
    def resizeEvent(self, event):
        """Handle resize event."""
        super().resizeEvent(event)
        self.position_control_widget()
    
    def position_control_widget(self):
        """Position control widget at bottom center of view."""
        if not self.control_widget:
            return
        
        # Position at bottom center with 20px margin
        size = self.control_widget.sizeHint()
        x = (self.width() - size.width()) // 2
        y = self.height() - size.height() - 20
        
        self.control_widget.move(x, y)
        
    def paintGL(self):
        """Render the scene."""
        self.draw_background()
        self.draw_robot()
        self.draw_trajectories()
        
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
            
    def draw_trajectories(self):
        """Draw robot trajectories."""
        if len(self.traj_points) > 1:
            self.traj_buffer.load(self.traj_points)
            
            with Program.Use(self.program, ["coord"]):
                self.program.set_attribute("coord", self.traj_buffer, components=2)
                self.program.set_uniform("color", 1.0, 0.0, 0.0)  # Red for odometry
                mat = np.eye(3, dtype=np.float32) * self.scale_factor
                self.program.set_uniform("mat", mat)
                glDrawArrays(GL_LINE_STRIP, 0, len(self.traj_points))
        
        if len(self.true_traj_points) > 1:
            self.true_traj_buffer.load(self.true_traj_points)
            
            with Program.Use(self.program, ["coord"]):
                self.program.set_attribute("coord", self.true_traj_buffer, components=2)
                self.program.set_uniform("color", 0.0, 0.0, 1.0)  # Blue for ground truth
                mat = np.eye(3, dtype=np.float32) * self.scale_factor
                self.program.set_uniform("mat", mat)
                glDrawArrays(GL_LINE_STRIP, 0, len(self.true_traj_points))
                
    def draw_robot(self):
        """Draw the robot at current position."""
        # Draw both robots - true first (bright), then odometry (faded)
        if self.robot_texture is not None:
            self.draw_robot_at_pose(self.true_heading, self.true_pos, fade=1.0)
            self.draw_robot_at_pose(self.curr_heading, self.curr_pos, fade=0.6)
        else:
            # Fallback: draw simple colored squares if texture not available
            self.draw_robot_simple(self.true_heading, self.true_pos, (0.0, 0.9, 0.1))
            self.draw_robot_simple(self.curr_heading, self.curr_pos, (0.9, 0.9, 0.9))
        
    def draw_robot_simple(self, heading: float, pos: np.ndarray, color: tuple):
        """Draw robot as a simple colored quad (for debugging)."""
        c = np.cos(heading)
        s = np.sin(heading)
        
        mat = np.array([
            [c * self.scale_factor, -s * self.scale_factor, pos[0] * self.scale_factor],
            [s * self.scale_factor, c * self.scale_factor, pos[1] * self.scale_factor],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        with Program.Use(self.program, ["coord"]):
            self.program.set_attribute("coord", self.robot_buffer, components=2)
            self.program.set_uniform("mat", mat)
            self.program.set_uniform("color", *color)
            
            # Draw robot as filled quad
            self.robot_index_buffer.bind()
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, None)
            self.robot_index_buffer.unbind()
    
    def draw_robot_at_pose(self, heading: float, pos: np.ndarray, fade: float):
        """Draw robot at specific pose."""
        # Create transformation matrix
        c = np.cos(heading)
        s = np.sin(heading)
        mat = np.array([
            [c * self.scale_factor, -s * self.scale_factor, pos[0] * self.scale_factor],
            [s * self.scale_factor, c * self.scale_factor, pos[1] * self.scale_factor],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        with Program.Use(self.texture_program, ["coord", "tex_coord"]):
            self.texture_program.set_attribute("coord", self.robot_buffer, components=2)
            self.texture_program.set_attribute("tex_coord", self.robot_tex_buffer, components=2)
            self.texture_program.set_uniform("mat", mat)
            self.texture_program.set_uniform("fade", fade)
            
            # Bind texture to unit 0
            self.robot_texture.bind(GL_TEXTURE0)
            # Tell shader to use texture unit 0
            self.texture_program.set_uniform("texture", 0)
            
            # Draw robot
            self.robot_index_buffer.bind()
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, None)
            self.robot_index_buffer.unbind()
            
            self.robot_texture.unbind(GL_TEXTURE0)
            
    def update_simulation(self):
        """Update simulation state."""
        dt = 0.025  # 25ms timestep
        
        # Get input from control widget or keyboard
        left_velocity = 0.0
        right_velocity = 0.0
        
        # Read joystick values from control widget
        if self.control_widget and self.control_widget.isVisible():
            forward = self.control_widget.left_value()   # 0.0 to 1.0
            turn = self.control_widget.right_value()      # -1.0 to +1.0
            
            # Calculate differential drive velocities
            # Scale to encoder ticks per sample (matching C++ version)
            k1 = 1.0
            k2 = 0.5
            left_vel = k1 * forward - k2 * turn
            right_vel = k1 * forward + k2 * turn
            
            # Scale to target encoder ticks (matching C++ version: 50.0 * value)
            target_left_vel = int(np.ceil(50.0 * left_vel))
            target_right_vel = int(np.ceil(50.0 * right_vel))
            
            self.left_wheel.set_target(target_left_vel)
            self.right_wheel.set_target(target_right_vel)
        else:
            # Fallback to keyboard input (scaled to encoder ticks per sample)
            target_fw_vel = 0
            target_twist_vel = 0
            
            if Qt.Key.Key_W in self.keys_pressed:
                target_fw_vel = 50  # Forward
            if Qt.Key.Key_S in self.keys_pressed:
                target_fw_vel = -50  # Backward
            if Qt.Key.Key_A in self.keys_pressed:
                target_twist_vel = -20  # Turn left
            if Qt.Key.Key_D in self.keys_pressed:
                target_twist_vel = 20  # Turn right
            
            self.left_wheel.set_target(target_fw_vel - target_twist_vel)
            self.right_wheel.set_target(target_fw_vel + target_twist_vel)
        
        # Update wheels (get velocity in encoder ticks per sample)
        left_ticks = self.left_wheel.update()
        right_ticks = self.right_wheel.update()
        
        # Forward kinematics (odometry)
        self.update_odometry(left_ticks, right_ticks)
        
        # True state (perfect encoder readings)
        left_true_dist = encoder_ticks_to_distance(left_ticks)
        right_true_dist = encoder_ticks_to_distance(right_ticks)
        self.update_true_state(left_true_dist, right_true_dist)
        
        # Record trajectory
        if len(self.traj_points) == 0 or \
           np.linalg.norm(self.curr_pos - np.array(self.traj_points[-1])) > 0.01:
            self.traj_points.append(self.curr_pos.tolist())
            if len(self.traj_points) > 1000:
                self.traj_points.pop(0)
                
        if len(self.true_traj_points) == 0 or \
           np.linalg.norm(self.true_pos - np.array(self.true_traj_points[-1])) > 0.01:
            self.true_traj_points.append(self.true_pos.tolist())
            if len(self.true_traj_points) > 1000:
                self.true_traj_points.pop(0)
        
        # Trigger redraw
        self.update()
        
    def update_odometry(self, left_ticks: float, right_ticks: float):
        """Update odometry estimate from encoder ticks."""
        # Convert ticks to distances
        left_dist = encoder_ticks_to_distance(left_ticks)
        right_dist = encoder_ticks_to_distance(right_ticks)
        
        # Differential drive forward kinematics
        distance = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / DISTANCE_BETWEEN_WHEELS
        
        # Update heading
        self.curr_heading += dtheta
        
        # Update position (midpoint integration)
        mid_heading = self.curr_heading - dtheta / 2.0
        self.curr_pos[0] += distance * np.cos(mid_heading)
        self.curr_pos[1] += distance * np.sin(mid_heading)
        
    def update_true_state(self, left_dist: float, right_dist: float):
        """Update true state (ground truth)."""
        distance = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / DISTANCE_BETWEEN_WHEELS
        
        self.true_heading += dtheta
        mid_heading = self.true_heading - dtheta / 2.0
        self.true_pos[0] += distance * np.cos(mid_heading)
        self.true_pos[1] += distance * np.sin(mid_heading)
        
    def keyPressEvent(self, event: QKeyEvent):
        """Handle key press."""
        self.keys_pressed.add(event.key())
        
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
            
    def keyReleaseEvent(self, event: QKeyEvent):
        """Handle key release."""
        self.keys_pressed.discard(event.key())
        
    def reset_simulation(self):
        """Reset simulation to initial state."""
        self.curr_heading = 0.0
        self.curr_pos = np.array([0.0, 0.0])
        self.true_heading = 0.0
        self.true_pos = np.array([0.0, 0.0])
        self.traj_points.clear()
        self.true_traj_points.clear()
        self.left_wheel.reset()
        self.right_wheel.reset()

