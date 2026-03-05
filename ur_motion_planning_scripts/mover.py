#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import ikpy.chain

class URMoveToConfig(Node):

    def __init__(self):
        super().__init__('ur_move_to_config')
        
        # The controller topic to publish to.
        # 'scaled_joint_trajectory_controller' is the default for the UR driver.
        # If using simulation without the scaled controller, you might need 
        # 'joint_trajectory_controller/joint_trajectory'.
        topic_name = '/scaled_joint_trajectory_controller/joint_trajectory'
        
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # Load the chain from URDF
        # NOTE: You must provide the correct path to your robot's URDF file.
        # The active_links_mask assumes a standard UR chain: Base(Fixed), 6 Joints, Tip(Fixed)
        self.urdf_path = 'ur3e.urdf' 
        self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path, active_links_mask=[False, False, True, True, True, True, True, True, False])

        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Subscription for target commands: Expects [x, y, z, velocity]
        # x, y, z in meters; velocity in m/s
        self.target_sub = self.create_subscription(Float64MultiArray, '/pendulum_target', self.target_callback, 10)

    def joint_state_callback(self, msg):
        # Map joint names to positions to ensure correct order
        # UR joints: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        try:
            name_map = {n: p for n, p in zip(msg.name, msg.position)}
            self.current_joints = [name_map[n] for n in order]
        except KeyError:
            pass

    def target_callback(self, msg):
        if self.current_joints is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return

        if len(msg.data) < 4:
            self.get_logger().error("Invalid target msg. Expected [x, y, z, velocity]")
            return

        # Parse target
        target_pos = np.array(msg.data[:3])
        velocity = msg.data[3]
        if velocity <= 0: velocity = 0.1 # Safety default

        # Target orientation: Pointing down (Tool Z aligned with World -Z)
        target_orientation = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])

        # Seed IK with current robot state to prevent jumps
        # Pad current 6 joints to match 9 links in chain (indices 2-7 are active)
        ik_seed = [0.0] * len(self.chain.links)
        ik_seed[2:8] = self.current_joints

        # Calculate Inverse Kinematics
        ik_solution = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orientation,
            orientation_mode="all",
            initial_position=ik_seed
        )
        
        # Extract the 6 active joints
        target_joints = list(ik_solution[2:8])

        # Safety check for configuration jumps
        max_diff = max([abs(t - c) for t, c in zip(target_joints, self.current_joints)])
        
        if max_diff > 0.5:
            # If we are far from the target, treat this as an approach move.
            # We ignore the requested velocity and move at a safe joint speed.
            safe_joint_vel = 0.5 # rad/s
            duration_sec = max_diff / safe_joint_vel
            self.get_logger().info(f"Approaching target (diff: {max_diff:.2f} rad). Duration: {duration_sec:.2f}s", throttle_duration_sec=1.0)
        else:
            # Calculate duration based on Cartesian distance and requested velocity
            # We use Forward Kinematics on the current joints to get the current Cartesian position
            current_fk = self.chain.forward_kinematics(ik_seed)
            current_pos = current_fk[:3, 3]
            
            dist = np.linalg.norm(target_pos - current_pos)
            
            # Ensure a minimum time step to prevent invalid acceleration faults
            # 0.02s is 50Hz, which is a safe lower bound for trajectory points
            duration_sec = max(dist / velocity, 0.02)

        # Construct and publish trajectory
        traj_msg = JointTrajectory()
        
        # Standard UR joint names. 
        # Note: If you used a tf_prefix in the launch file, these names might need that prefix.
        traj_msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = URMoveToConfig()
    
    # Spin once to ensure the message is sent, or keep spinning if you want to listen for feedback
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()