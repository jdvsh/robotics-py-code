import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        # The topic name usually defaults to scaled_joint_trajectory_controller for UR robots
        # If this doesn't work, try '/joint_trajectory_controller/joint_trajectory'
        topic_name = '/scaled_joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        # Run at 10Hz (0.1 seconds) for continuous control
        # For a real pendulum, you might need to go faster (e.g. 0.02s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        msg = JointTrajectory()
        # Standard UR3e joint names
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        # Target positions in radians
        wrist_command = math.sin(elapsed * 2.0)
        point.positions = [wrist_command, -1.57, 1.0, 0.0, 0.0, wrist_command]
        
        # Tell the controller to reach this state in 0.1 seconds (immediate streaming)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 1000000000 # 100ms
        
        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()