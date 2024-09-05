import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.command_publisher_ = self.create_publisher(String, '/franka_control/move_command', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/franka_control/target_pose', 10)
        self.force_feedback_subscription = self.create_subscription(
            JointState, '/franka_state_controller/joint_states', self.force_callback, 10)
        self.current_force = 0.0

    def send_command(self, action):
        msg = String()
        msg.data = action
        self.command_publisher_.publish(msg)
        self.get_logger().info(f'Sent command to robot: {action}')

    def poke_object(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0
        
        self.pose_publisher_.publish(target_pose)
        time.sleep(1)  # Simulate time spent poking
        return self.current_force
    
    def reset_position(self):
        # Send a command to reset the robot to its default position
        self.send_command("reset_position")
        self.get_logger().info("Reset command sent. Waiting for robot to reach default position...")
        time.sleep(5)  # Wait for the robot to reach the default position (adjust time as needed)

    def force_callback(self, msg):
        # Update current force based on the joint state data
        self.current_force = sum(abs(torque) for torque in msg.effort)
        self.get_logger().info(f"Current force: {self.current_force} N")

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    
    # Example usage
    robot_control.reset_position()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
