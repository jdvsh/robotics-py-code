#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from scipy.spatial.transform import Rotation
import numpy as np

class URMoveLinear(Node):

    def __init__(self):
        super().__init__('ur_move_linear')
        
        # Topic to send raw URScript commands directly to the robot
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

        # Define the robot base position in the world frame [x, y, z]
        self.robot_base_pos = np.array([0.0, 0.0, 0.0])

        # Subscription for target commands: Expects [x, y, z, velocity]
        self.target_sub = self.create_subscription(Float64MultiArray, '/pendulum_target', self.target_callback, 10)

        # Your fixed orientation matrix
        self.target_orientation_matrix = np.array([
            [0, 1, 0], 
            [0, 0, 1], 
            [1, 0, 0]
        ])
        
        # URScript expects orientation as an axis-angle rotation vector (rx, ry, rz).
        # We can calculate this once using scipy.
        self.rotvec = Rotation.from_matrix(self.target_orientation_matrix).as_rotvec()

    def target_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().error("Invalid target msg. Expected [x, y, z, velocity]")
            return

        # Parse target and convert to robot-relative coordinates
        target_pos_world = np.array(msg.data[:3])
        target_pos = target_pos_world - self.robot_base_pos
        velocity = msg.data[3]
        if velocity <= 0: velocity = 0.1 # Safety default
        
        acceleration = 1.2 # Standard UR acceleration (m/s^2)

        # Construct the URScript movel command
        # Format: movel(p[x, y, z, rx, ry, rz], a=acceleration, v=velocity)
        # Note: URScript expects meters and radians.
        script_cmd = f"movel(p[{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}, {self.rotvec[0]:.4f}, {self.rotvec[1]:.4f}, {self.rotvec[2]:.4f}], a={acceleration}, v={velocity})\n"

        # Publish the command
        msg_str = String()
        msg_str.data = script_cmd
        self.script_pub.publish(msg_str)
        self.get_logger().info(f"Sent Linear Move: {script_cmd.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = URMoveLinear()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()