import rclpy
from robot_control import RobotControl

def test_poke_objects():
    rclpy.init()
    robot_control = RobotControl()

    try:
        # Move to the first object and poke it
        print("Moving to the first object...")
        # Example joint positions for the first object
        joint_positions_1 = [0.5, 0.0, 0.3, 0, 0, 0, 0, 0, 0]  # Update as needed
        force_1 = robot_control.poke_object(joint_positions_1)
        print(f"Force measured at first object: {force_1} N")
        
        # Reset position
        print("Resetting position...")
        robot_control.reset_position()
        
        # Move to the second object and poke it
        print("Moving to the second object...")
        # Example joint positions for the second object
        joint_positions_2 = [0.6, 0.0, 0.4, 0, 0, 0, 0, 0, 0]  # Update as needed
        force_2 = robot_control.poke_object(joint_positions_2)
        print(f"Force measured at second object: {force_2} N")
        
        # Compare forces
        if force_1 > force_2:
            print("The first object is harder.")
        elif force_1 < force_2:
            print("The second object is harder.")
        else:
            print("Both objects have the same hardness.")

    except KeyboardInterrupt:
        print("Test interrupted.")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    test_poke_objects()
