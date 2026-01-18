#!/usr/bin/env python3
"""
Inverse Kinematics Simulation for Romi Robot using PyBullet.
Robot follows hardcoded waypoints using PID-based motion control.

Requirements:
    pip install pybullet numpy

Usage:
    # From pybullet_simulation directory:
    python inverse_kinematics.py
    
    # Or from workspace root as module:
    python -m pybullet_simulation.inverse_kinematics
"""

import os
import sys
import time
import pybullet as p
import pybullet_data
import numpy as np

# Add workspace root to path for package imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_simulations.robot_models import ENCODER_TICKS_PER_ROTATION
from motion_controller import MotionController

# Some pybullet builds don't define B3G_ESCAPE; fall back to ASCII ESC (27)
ESC_KEY = getattr(p, "B3G_ESCAPE", 27)

# Control loop period (25ms = 40 Hz)
CONTROL_DT = 0.025  # 25ms


def encoder_ticks_per_sample_to_rad_per_sec(ticks_per_sample: int) -> float:
    """
    Convert encoder ticks per sample (25ms) to angular velocity in rad/s.
    
    Args:
        ticks_per_sample: Encoder ticks per sample (25ms period)
        
    Returns:
        Angular velocity in radians per second
    """
    # Convert ticks per sample to ticks per second
    ticks_per_sec = ticks_per_sample / CONTROL_DT
    # Convert to rotations per second
    rotations_per_sec = ticks_per_sec / ENCODER_TICKS_PER_ROTATION
    # Convert to rad/s
    rad_per_sec = rotations_per_sec * (2.0 * np.pi)
    return rad_per_sec


class RomiSimulator:
    def __init__(self, waypoints: list):
        """
        Initialize PyBullet simulation.
        
        Args:
            waypoints: List of (x, y) tuples for waypoints in meters
        """
        # Connect to PyBullet GUI
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Configure debug visualizer to show only RGB (disable depth/segmentation views)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  # Disable depth preview
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  # Disable segmentation preview
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)  # Enable RGB preview only
        
        # Set up camera
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=0,
            cameraPitch=-60,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Physics setup
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 200.0)  # 200 Hz timestep
        
        # Load stadium ground (SDF format)
        # loadSDF returns a list of body IDs
        stadium_ids = p.loadSDF("stadium.sdf")
        if stadium_ids and len(stadium_ids) > 0:
            self.plane_id = stadium_ids[0]  # Use first body ID
        
        # Load Romi robot (mesh-based URDF)
        urdf_path = os.path.join(os.path.dirname(__file__), "romi_meshes.urdf")
        if not os.path.exists(urdf_path):
            print(f"ERROR: URDF file not found at {urdf_path}")
            sys.exit(1)
            
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.05],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False
        )
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_info = {}
        self.left_wheel_idx = None
        self.right_wheel_idx = None
        self.camera_link_index = None
        
        print("\n=== Robot Joint Information ===")
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            
            self.joint_info[joint_name] = {
                'index': i,
                'type': joint_type,
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }
            
            print(f"Joint {i}: {joint_name} (Type: {joint_type})")
            
            # Find wheel joints
            if joint_name == "left_wheel_joint":
                self.left_wheel_idx = i
            elif joint_name == "right_wheel_joint":
                self.right_wheel_idx = i
            # Find camera link (child link of camera_joint)
            elif joint_name == "camera_joint":
                # In PyBullet, link index for a joint's child link is the joint index
                # But we need to verify - try using the joint index itself
                self.camera_link_index = i  # Use joint index as link index
                child_link_index = info[16]  # childLinkIndex from joint info
                print(f"  -> Camera joint index: {i}, childLinkIndex: {child_link_index}")
                print(f"  -> Using camera link index: {self.camera_link_index}")
        
        print(f"\nLeft wheel index: {self.left_wheel_idx}")
        print(f"Right wheel index: {self.right_wheel_idx}")
        if self.camera_link_index is not None:
            print(f"Camera link index: {self.camera_link_index}")
        else:
            print("Camera link not found")
        
        # Initialize motion controller
        self.controller = MotionController()
        
        # Add waypoints to controller
        # Convert waypoints from PyBullet coordinates (Y left) to controller coordinates (Y right)
        # PyBullet: X forward, Y left, Z up
        # Controller: X forward, Y right, Z up
        # Conversion: negate Y coordinate when storing in controller
        # Note: waypoints parameter is in PyBullet coordinates (as defined in main())
        for x, y in waypoints:
            self.controller.add_waypoint(x, -y)  # Negate Y to convert from PyBullet to controller coords
        
        # Set first waypoint as current destination and remove it from queue
        if self.controller.waypoints:
            self.controller.current_dest = self.controller.waypoints.pop(0)
            print(f"\nInitial destination: ({self.controller.current_dest.pos[0]:.3f}, {self.controller.current_dest.pos[1]:.3f})")
        
        # Trajectory tracking for visualization
        self.trajectory = []  # List of (x, y, z) positions
        self.trajectory_line_ids = []  # List of line IDs for drawing
        self.trajectory_update_interval = 0.1  # Update trajectory every 0.1 seconds
        self.trajectory_update_accumulator = 0.0
        
        # Waypoint visualization
        self.waypoint_line_ids = []
        self.draw_waypoints(waypoints)
        
        # Time tracking for control loop (25ms intervals)
        self.control_update_accumulator = 0.0
        self.pybullet_dt = 1.0 / 200.0  # PyBullet timestep (200 Hz)
        
        # Camera settings - Raspberry Pi Camera Module 2 specifications
        self.camera_enabled = self.camera_link_index is not None
        # Using 640x480 for better rendering performance (maintains 4:3 aspect ratio like native 3280x2464)
        self.camera_width = 640
        self.camera_height = 480
        # Horizontal FOV: ~62.2 degrees (calculated from sensor size 3.68mm, focal length 3.04mm)
        # PyBullet's computeProjectionMatrixFOV uses horizontal FOV
        self.camera_fov = 62.2  # degrees (horizontal FOV)
        self.camera_update_interval = 0.1  # Capture at 10 Hz
        self.camera_update_accumulator = 0.0
        self.last_camera_image = None
        
        print("\n=== Inverse Kinematics Simulation ===")
        print(f"Number of waypoints: {len(waypoints)}")
        print("ESC: Exit")
        print("\nTrajectory is drawn in red as the robot moves.")
        print("Waypoints are drawn in blue.")
        if self.camera_enabled:
            print("Camera enabled - capturing images from camera_link")
        print("Simulation started...")
    
    def draw_circle(self, center_x: float, center_y: float, radius: float = 0.02, 
                    z: float = 0.05, color: list = [0.0, 0.0, 1.0], 
                    line_width: int = 2, num_segments: int = 16, 
                    life_time: float = 0) -> list:
        """
        Draw a circle on the xy plane using line segments.
        
        Args:
            center_x: X coordinate of circle center
            center_y: Y coordinate of circle center
            radius: Circle radius in meters (default: 0.02)
            z: Z coordinate (height) in meters (default: 0.05)
            color: RGB color as list [r, g, b] (default: blue)
            line_width: Line width (default: 2)
            num_segments: Number of line segments to approximate circle (default: 16)
            life_time: Line lifetime in seconds, 0 means permanent (default: 0)
            
        Returns:
            List of line IDs created for the circle
        """
        line_ids = []
        prev_point = None
        
        for i in range(num_segments + 1):
            angle = 2.0 * np.pi * i / num_segments
            px = center_x + radius * np.cos(angle)
            py = center_y + radius * np.sin(angle)
            current_point = [px, py, z]
            
            if prev_point is not None:
                line_id = p.addUserDebugLine(
                    prev_point,
                    current_point,
                    lineColorRGB=color,
                    lineWidth=line_width,
                    lifeTime=life_time
                )
                line_ids.append(line_id)
            prev_point = current_point
        
        return line_ids
    
    def draw_waypoints(self, waypoints: list):
        """
        Draw waypoints as blue lines and circles.
        
        Args:
            waypoints: List of (x, y) tuples in PyBullet coordinates (original waypoints)
        """
        if len(waypoints) < 2:
            return
        
        # Draw lines connecting waypoints (in PyBullet coordinates)
        for i in range(len(waypoints) - 1):
            start = [waypoints[i][0], waypoints[i][1], 0.05]
            end = [waypoints[i+1][0], waypoints[i+1][1], 0.05]
            line_id = p.addUserDebugLine(
                start,
                end,
                lineColorRGB=[0.0, 0.0, 1.0],  # Blue color
                lineWidth=2,
                lifeTime=0  # 0 means permanent
            )
            self.waypoint_line_ids.append(line_id)
        
        # Draw waypoint markers (small circles on xy plane) in PyBullet coordinates
        for x, y in waypoints:
            circle_line_ids = self.draw_circle(
                center_x=x,
                center_y=y,
                radius=0.02,  # 2cm radius
                z=0.05,
                color=[0.0, 0.0, 1.0],  # Blue color
                line_width=2,
                num_segments=16
            )
            self.waypoint_line_ids.extend(circle_line_ids)
    
    def set_wheel_velocities(self, left_ticks, right_ticks):
        """
        Apply wheel velocities to PyBullet joints.
        
        Args:
            left_ticks: Left wheel velocity in encoder ticks per sample
            right_ticks: Right wheel velocity in encoder ticks per sample
        """
        # Convert encoder ticks per sample to rad/s
        left_vel = encoder_ticks_per_sample_to_rad_per_sec(left_ticks)
        right_vel = encoder_ticks_per_sample_to_rad_per_sec(right_ticks)
        
        if self.left_wheel_idx is not None:
            p.setJointMotorControl2(
                self.robot_id,
                self.left_wheel_idx,
                p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=1.0
            )
        
        if self.right_wheel_idx is not None:
            p.setJointMotorControl2(
                self.robot_id,
                self.right_wheel_idx,
                p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=1.0
            )
    
    def get_camera_image(self):
        """
        Capture an image from the camera link's perspective.
        
        Returns:
            rgb_array: RGB image as numpy array (height, width, 3), or None if camera not available
        """
        if not self.camera_enabled or self.camera_link_index is None:
            return None
        
        # Get camera link pose (position and orientation)
        link_state = p.getLinkState(self.robot_id, self.camera_link_index, computeForwardKinematics=True)
        camera_pos = link_state[0]  # Position [x, y, z]
        camera_orn = link_state[1]  # Orientation as quaternion [x, y, z, w]
        
        
        # Get rotation matrix from quaternion
        # Matrix is stored as [m00, m01, m02, m10, m11, m12, m20, m21, m22]
        rotation_matrix = p.getMatrixFromQuaternion(camera_orn)
        
        # In URDF/PyBullet: X forward, Y left, Z up
        # Camera looks along +X axis (forward direction)
        # Camera up is +Z axis
        forward_local = np.array([1.0, 0.0, 0.0])  # Camera looks forward along X
        up_local = np.array([0.0, 0.0, 1.0])  # Camera up is Z
        
        # Transform to world coordinates
        # Rotation matrix columns are the transformed unit vectors
        # [m00 m01 m02]   [1]   [m00]
        # [m10 m11 m12] * [0] = [m10]
        # [m20 m21 m22]   [0]   [m20]
        forward_world = np.array([
            rotation_matrix[0],  # First column (X-axis of camera frame)
            rotation_matrix[3],
            rotation_matrix[6]
        ])
        up_world = np.array([
            rotation_matrix[2],  # Third column (Z-axis of camera frame)
            rotation_matrix[5],
            rotation_matrix[8]
        ])
        
        # Normalize
        forward_world = forward_world / np.linalg.norm(forward_world)
        up_world = up_world / np.linalg.norm(up_world)
        
        # Compute target position (camera looks forward, 2 meters ahead)
        target_pos = camera_pos + forward_world * 2.0
        
        # Compute view matrix
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_world
        )
        
        # Projection matrix - Raspberry Pi Camera Module 2 specifications
        # Aspect ratio: 4:3 (matches native 3280x2464 resolution)
        aspect = self.camera_width / self.camera_height  # Should be ~1.333 (4:3)
        # Near and far planes
        near = 0.0125  # 1.25cm - avoids rendering camera's own geometry
        far = 100.0    # 100m - sufficient for most simulation scenarios
        # computeProjectionMatrixFOV uses horizontal FOV (matches Raspberry Pi Camera Module 2: ~62.2°)
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,  # Horizontal FOV in degrees
            aspect=aspect,
            nearVal=near,
            farVal=far
        )
        
        # Render image - only RGB data is captured and used
        # Depth and segmentation are completely disabled
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.camera_width,
            height=self.camera_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            flags=p.ER_NO_SEGMENTATION_MASK  # Disable segmentation mask computation
        )
        
        # Explicitly ignore depth and segmentation (they may still be computed but not used)
        # Note: PyBullet may still compute depth internally, but we don't use it
        del depth_img, seg_img
        
        # Extract RGB channels only (remove alpha channel)
        rgb_array = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
        
        return rgb_array
    
    def update_trajectory(self):
        """Update trajectory with current robot position from PyBullet."""
        # Get position from PyBullet (use PyBullet coordinates directly for trajectory)
        pb_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        pos = [pb_pos[0], pb_pos[1], pb_pos[2]]  # Use PyBullet coordinates directly
        
        # Add new point if trajectory is empty or if moved significantly
        if len(self.trajectory) == 0:
            self.trajectory.append(pos)
        else:
            # Only add point if moved more than 1cm
            last_pos = self.trajectory[-1]
            dx = pos[0] - last_pos[0]
            dy = pos[1] - last_pos[1]
            dz = pos[2] - last_pos[2]
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            if distance > 0.01:  # 1cm threshold
                # Draw line from last point to current point
                line_id = p.addUserDebugLine(
                    (last_pos[0], last_pos[1], 0.05),
                    (pos[0], pos[1], 0.05),
                    lineColorRGB=[1.0, 0.0, 0.0],  # Red color
                    lineWidth=2,
                    lifeTime=0  # 0 means permanent
                )
                self.trajectory_line_ids.append(line_id)
                self.trajectory.append(pos)
    
    def run(self):
        """Main simulation loop."""
        while True:
            # Check for ESC key
            keys = p.getKeyboardEvents()
            if ESC_KEY in keys and keys[ESC_KEY] & p.KEY_WAS_TRIGGERED:
                print("\nExiting simulation...")
                break
            
            # Update control loop at 25ms intervals (40 Hz)
            self.control_update_accumulator += self.pybullet_dt
            
            # Step simulation first to update physics
            p.stepSimulation()
            
            if self.control_update_accumulator >= CONTROL_DT:
                # Get current robot pose from PyBullet physics (after step)
                pb_pos, pb_orn = p.getBasePositionAndOrientation(self.robot_id)
                pb_euler = p.getEulerFromQuaternion(pb_orn)
                
                # Convert PyBullet coordinates to controller coordinates
                # PyBullet: X forward, Y left, Z up
                # Controller: X forward, Y right, Z up
                # Conversion: negate Y coordinate and yaw
                pos_x = pb_pos[0]  # X stays the same
                pos_y = -pb_pos[1]  # Negate Y (left to right)
                heading = -pb_euler[2]  # Negate yaw for coordinate system conversion
                
                # Update motion controller with PyBullet position (converted to controller coords)
                left_ticks, right_ticks = self.controller.update(
                    pos_x=pos_x,
                    pos_y=pos_y,
                    heading=heading,
                    dt=CONTROL_DT
                )
                
                # Apply wheel velocities
                self.set_wheel_velocities(left_ticks, right_ticks)
            
                self.control_update_accumulator -= CONTROL_DT
            
            # Update trajectory periodically
            self.trajectory_update_accumulator += self.pybullet_dt
            if self.trajectory_update_accumulator >= self.trajectory_update_interval:
                self.update_trajectory()
                self.trajectory_update_accumulator = 0.0
            
            # Capture camera image periodically
            if self.camera_enabled:
                self.camera_update_accumulator += self.pybullet_dt
                if self.camera_update_accumulator >= self.camera_update_interval:
                    self.last_camera_image = self.get_camera_image()
                    if self.last_camera_image is not None:
                        # Debug: Print image stats
                        img_mean = np.mean(self.last_camera_image)
                        if img_mean < 1.0:  # Very dark image (likely empty)
                            print(f"Warning: Camera image appears empty (mean value: {img_mean:.2f})")
                        # Image is now available in self.last_camera_image
                        # You can process it, save it, or publish it to ROS2 here
                    self.camera_update_accumulator = 0.0
            
            #time.sleep(self.pybullet_dt)
        
        p.disconnect()


def main():
    """Entry point for the inverse kinematics simulation."""
    print("Pololu Romi Robot - Inverse Kinematics Simulation")
    
    # Hardcoded waypoints (x, y) in meters (defined in PyBullet coordinate system)
    waypoints = [
        (0.5, 0.0),
        (0.0, 0.5),
        (-0.5, 0.0),
        (0.0, -0.5),
        (0.5, 0.0),
        (0.0, 0.0)
    ]
    
    try:
        simulator = RomiSimulator(waypoints)
        simulator.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Simulation ended.")


if __name__ == "__main__":
    main()
