import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.trajectory_publisher_ = self.create_publisher(JointTrajectory, '/franka_control/joint_trajectory', 10)
        self.force_feedback_subscription = self.create_subscription(
            JointState, '/franka_state_controller/joint_states', self.force_callback, 10)
        self.current_force = 0.0

    def send_trajectory(self, joint_positions):
        # Debug print statement
        print(f"Debug: Joint positions received: {joint_positions}")
        
        # Ensure joint_positions is a list of floats with correct length
        if not isinstance(joint_positions, (list, tuple)):
            self.get_logger().error("joint_positions must be a list or tuple.")
            return
        
        if len(joint_positions) != 9:  # Adjust based on the number of joints
            self.get_logger().error(f"joint_positions length must be 9, but got {len(joint_positions)}.")
            return

        if not all(isinstance(pos, (float, int)) for pos in joint_positions):
            self.get_logger().error("All joint_positions values must be of type float.")
            return
        
        # Convert to float to ensure all values are float
        joint_positions = [float(pos) for pos in joint_positions]
        
        trajectory = JointTrajectory()
        trajectory.header.frame_id = "base_link"
        trajectory.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7', 'fr3_finger_joint1',
            'fr3_finger_joint2'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2  # Give enough time for the robot to move
        
        trajectory.points.append(point)
        
        self.trajectory_publisher_.publish(trajectory)
        self.get_logger().info(f'Sent trajectory to robot: {joint_positions}')
    
    def poke_object(self, joint_positions):
        self.send_trajectory(joint_positions)
        time.sleep(5)  # Wait for the robot to complete the movement
        return self.current_force

    def reset_position(self):
        # Send a command to reset the robot to its default position
        self.send_trajectory([0.0] * 9)  # Example: Move all joints to 0.0 position
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
    joint_positions_1 = [0.5, 0.0, 0.3, 0, 0, 0, 0, 0, 0]
    joint_positions_2 = [0.6, 0.0, 0.4, 0, 0, 0, 0, 0, 0]
    
    robot_control.reset_position()
    force_1 = robot_control.poke_object(joint_positions_1)
    print(f"Force measured at first object: {force_1} N")
    
    robot_control.reset_position()
    force_2 = robot_control.poke_object(joint_positions_2)
    print(f"Force measured at second object: {force_2} N")
    
    if force_1 > force_2:
        print("The first object is harder.")
    elif force_2 > force_1:
        print("The second object is harder.")
    else:
        print("Both objects have the same hardness.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
