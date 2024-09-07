import PyKDL as kdl
import numpy as np
from urdf_parser_py.urdf import URDF


class FrankaIKSolver:
    def __init__(self, urdf_file):
        # Load the URDF model from the file
        robot = URDF.from_xml_file(urdf_file)
        
        # Base and end-effector link names, based on the URDF
        self.base_link = 'base'  # Matches the 'base' link in the URDF
        self.end_effector_link = 'fr3_hand'  # Matches the 'fr3_hand' link in the URDF

        # Build the KDL Tree from the URDF robot model
        self.kdl_tree = self.build_kdl_tree(robot)

        # Create the KDL chain from base link to end-effector link
        self.kdl_chain = self.kdl_tree.getChain(self.base_link, self.end_effector_link)
        
        # Create a KDL solver for forward kinematics
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)
        
        # Create a KDL solver for the Jacobian
        self.jac_solver = kdl.ChainJntToJacSolver(self.kdl_chain)

    def build_kdl_tree(self, robot):
        """
        Build the KDL Tree from the URDF model.
        :param robot: URDF robot model.
        :return: KDL tree.
        """
        kdl_tree = kdl.Tree(robot.get_root())
        for joint in robot.joints:
            if joint.joint_type == 'fixed':
                continue
            kdl_tree.addSegment(kdl.Segment(joint.name), kdl_tree.getSegment(joint.parent))
        return kdl_tree

    def forward_kinematics(self, joints):
        """
        Compute forward kinematics using PyKDL given joint angles.
        :param joints: Joint angles (7 DOF).
        :return: End-effector pose as a 4x4 homogeneous transformation matrix.
        """
        joint_array = kdl.JntArray(len(joints))
        for i, joint_angle in enumerate(joints):
            joint_array[i] = joint_angle

        # Calculate the forward kinematics
        end_effector_frame = kdl.Frame()
        self.fk_solver.JntToCart(joint_array, end_effector_frame)
        
        # Convert KDL frame to 4x4 numpy matrix
        pose_matrix = np.identity(4)
        for i in range(3):
            for j in range(3):
                pose_matrix[i, j] = end_effector_frame.M[i, j]
            pose_matrix[i, 3] = end_effector_frame.p[i]

        return pose_matrix

    def jacobian(self, joints):
        """
        Compute the Jacobian matrix using PyKDL for the given joint angles.
        :param joints: Joint angles (7 DOF).
        :return: Jacobian matrix (6x7).
        """
        joint_array = kdl.JntArray(len(joints))
        for i, joint_angle in enumerate(joints):
            joint_array[i] = joint_angle

        jacobian = kdl.Jacobian(len(joints))
        self.jac_solver.JntToJac(joint_array, jacobian)
        
        # Convert the KDL Jacobian to a numpy array
        jacobian_matrix = np.zeros((6, len(joints)))
        for i in range(6):
            for j in range(len(joints)):
                jacobian_matrix[i, j] = jacobian[i, j]

        return jacobian_matrix

    def inverse_kinematics(self, target_pose, initial_joints, max_iterations=1000, threshold=1e-3):
        """
        Solve the inverse kinematics for the given target pose.
        :param target_pose: Desired end-effector pose (4x4 homogeneous transformation matrix).
        :param initial_joints: Initial joint configuration (7 DOF).
        :param max_iterations: Maximum number of iterations for the IK solver.
        :param threshold: Threshold for convergence.
        :return: Solution joint angles if successful, else None.
        """
        joints = np.copy(initial_joints)

        for i in range(max_iterations):
            current_pose = self.forward_kinematics(joints)
            position_error = target_pose[:3, 3] - current_pose[:3, 3]
            orientation_error = 0.5 * (np.cross(current_pose[:3, 0], target_pose[:3, 0]) +
                                       np.cross(current_pose[:3, 1], target_pose[:3, 1]) +
                                       np.cross(current_pose[:3, 2], target_pose[:3, 2]))
            error = np.hstack((position_error, orientation_error))

            if np.linalg.norm(error) < threshold:
                return joints

            J = self.jacobian(joints)
            J_pseudo_inv = np.linalg.pinv(J)
            delta_joints = J_pseudo_inv @ error
            joints += delta_joints

        return None  # No solution found


# Example usage:
if __name__ == '__main__':
    # Define the URDF file path (update this with your actual URDF path)
    urdf_file_path = '/home/charisma/franka_ros2_ws/src/franka_description/robots/fr3/fr3.urdf'
    
    # Initialize the IK solver with the URDF
    ik_solver = FrankaIKSolver(urdf_file_path)
    
    # Define the target pose (4x4 matrix) and initial joint configuration (7 DOF)
    target_pose = np.identity(4)
    target_pose[0, 3] = 0.4  # Set x position
    target_pose[1, 3] = 0.2  # Set y position
    target_pose[2, 3] = 0.5  # Set z position
    
    initial_joints = np.zeros(7)  # Initial joint configuration
    
    # Solve IK
    solution_joints = ik_solver.inverse_kinematics(target_pose, initial_joints)
    
    if solution_joints is not None:
        print(f"Inverse kinematics solution found: {solution_joints}")
    else:
        print("No solution found.")
