import rclpy
from robot_control import RobotControl

def test_robot_control():
    # Initialize ROS client library
    rclpy.init()

    # Initialize RobotControl component
    robot_control = RobotControl()

    try:
        while True:
            # Prompt user for command
            command = input("Enter command (poke object, reset position, close program): ").strip().lower()
            
            if command == "poke object":
                print("Poking object...")
                force = robot_control.poke_object(x=0.5, y=0.0, z=0.3)
                print(f"Measured force: {force}")
                
            elif command == "reset position":
                print("Resetting position...")
                robot_control.reset_position()
                print("Position reset.")
                
            elif command == "close program":
                print("Closing program...")
                break  # Exit the loop and terminate the program
                
            else:
                print("Command not recognized. Please try again.")
    
    except KeyboardInterrupt:
        print("Test interrupted.")
    
    finally:
        # Cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    test_robot_control()
