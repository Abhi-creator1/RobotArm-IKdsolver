import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver')
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.target_position_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        # Initialize current joint angles
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0]
        # Joint limits
        self.lower_limits = [-np.pi, -np.pi / 2, -np.pi / 2, -np.pi]
        self.upper_limits = [np.pi, np.pi / 2, np.pi / 2, np.pi]
        # Parameters for IK
        self.learning_rate = 0.1
        self.tolerance = 1e-6
        self.max_iterations = 50
        self.get_logger().info("IK Solver Node initialized.")

    def target_position_callback(self, msg):
        # Extract target position
        x_target = msg.x
        y_target = msg.y
        z_target = msg.z

        # Call the IK solver
        try:
            joint_angles = self.inverse_kinematics(x_target, y_target, z_target)

            # Validate joint angles
            if not all(np.isfinite(joint_angles)):
                raise ValueError("Computed joint angles contain invalid values (e.g., NaN or Inf).")

            # Create and publish the JointState message
            joint_state = JointState()
            joint_state.name = ['world_to_baselink', 'base_to_shoulder', 'shoulder_to_elbow', 'elbow_to_wrist1']
            joint_state.position = [float(angle) for angle in joint_angles]
            joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(joint_state)

            self.get_logger().info(f"Published joint angles: {joint_angles}")
        except Exception as e:
            self.get_logger().error(f"Error in IK Solver: {e}")

    def clamp_joint_values(self, joint_positions):
        """
        Clamp joint positions within their respective limits.
        """
        return np.clip(joint_positions, self.lower_limits, self.upper_limits)

    def inverse_kinematics(self, x_target, y_target, z_target):
        # Define DH parameters
        a1, alpha1, d1 = 0, np.pi / 2, 0.14415
        a2, alpha2, d2 = 0.24381, 0, 0
        a3, alpha3, d3 = 0.12475, 0, 0
        a4, alpha4, d4 = 0.08475, -np.pi / 2, 0


        # Helper functions
        def dh_transform(a, alpha, d, theta):
            return np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])

        def forward_kinematics(theta1, theta2, theta3, theta4):
            T1 = dh_transform(a1, alpha1, d1, theta1)
            T2 = dh_transform(a2, alpha2, d2, theta2)
            T3 = dh_transform(a3, alpha3, d3, theta3)
            T4 = dh_transform(a4, alpha4, d4, theta4)
            T = np.matmul(np.matmul(np.matmul(T1, T2), T3), T4)
            return T[:3, 3]

        # Use stored joint angles as the initial guess
        theta = np.array(self.current_joint_angles, dtype=float)

        for iteration in range(self.max_iterations):
            # Current end-effector position
            position = forward_kinematics(*theta)
            error = np.linalg.norm(position - np.array([x_target, y_target, z_target]))

            if error < self.tolerance:
                self.get_logger().info(f"Converged after {iteration} iterations with error: {error}")
                break

            # Compute Jacobian numerically
            jacobian = np.zeros((3, 4))
            for i in range(4):
                delta = 1e-6
                theta_copy = theta.copy()
                theta_copy[i] += delta
                position_delta = forward_kinematics(*theta_copy) - position
                jacobian[:, i] = position_delta / delta

            try:
                # Compute the position error
                position_error = np.array([x_target, y_target, z_target]) - position

                # Solve for joint angle updates using the pseudo-inverse
                delta_theta = np.linalg.pinv(jacobian) @ position_error

                # Scale the updates
                delta_theta = np.clip(delta_theta, -0.1, 0.1)  # Limit max step size
                theta += self.learning_rate * delta_theta

                # Clamp the joint angles within limits
                theta = self.clamp_joint_values(theta)
            except np.linalg.LinAlgError as e:
                self.get_logger().error(f"Jacobian pseudo-inverse failed: {e}")
                break

        # Save the updated joint angles
        self.current_joint_angles = theta.tolist()
        return theta

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
