import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver')
        self.subscription = self.create_subscription(Point, '/target_position', self.target_position_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.current_joint_angles = np.array([0.0, np.pi / 4, -np.pi / 4, 0.0])  
        self.lower_limits = np.array([-np.pi, -np.pi / 2, -np.pi / 2, -np.pi])
        self.upper_limits = np.array([np.pi, np.pi / 2, np.pi / 2, np.pi])
        self.learning_rate = 0.02
        self.tolerance = 1e-6
        self.max_iterations = 80

    def target_position_callback(self, msg):
        x_target, y_target, z_target = msg.x, msg.y, msg.z
        joint_angles, final_error = self.inverse_kinematics(x_target, y_target, z_target)

        if not all(np.isfinite(joint_angles)):
            return
        
        joint_state = JointState()
        joint_state.name = ['world_to_baselink', 'base_to_shoulder', 'shoulder_to_elbow', 'elbow_to_wrist1']
        joint_state.position = [float(angle) for angle in joint_angles]
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(joint_state)

        print(f"Final Joint Angles: {joint_angles}")
        print(f"Final Error: {final_error}")

    def clamp_joint_values(self, joint_positions):
        return np.clip(joint_positions, self.lower_limits, self.upper_limits)

    def inverse_kinematics(self, x_target, y_target, z_target):
        a1, alpha1, d1 = 0, np.pi / 2, 0.14415
        a2, alpha2, d2 = 0.24381, 0, 0
        a3, alpha3, d3 = 0.12475, 0, 0
        a4, alpha4, d4 = 0.08475, -np.pi / 2, 0

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
            T = T1 @ T2 @ T3 @ T4
            return T[:3, 3]

        theta = np.array(self.current_joint_angles, dtype=float)

        for _ in range(self.max_iterations):
            position = forward_kinematics(*theta)
            error = np.linalg.norm(position - np.array([x_target, y_target, z_target]))

            if error < self.tolerance:
                break

            jacobian = np.zeros((3, 4))
            delta = 1e-6
            for i in range(4):
                theta_copy = theta.copy()
                theta_copy[i] += delta
                position_delta = forward_kinematics(*theta_copy) - position
                jacobian[:, i] = position_delta / delta

            try:
                position_error = np.array([x_target, y_target, z_target]) - position
                jacobian_norm = np.linalg.norm(jacobian)
                damping_factor = 0.01  
                if jacobian_norm < 1e-6:
                    jacobian += damping_factor * np.eye(jacobian.shape[0])

                delta_theta = np.linalg.pinv(jacobian) @ position_error
                adaptive_learning_rate = self.learning_rate * (1.0 / (1.0 + error))
                delta_theta = np.clip(delta_theta, -0.2, 0.2)
                theta += adaptive_learning_rate * delta_theta
                theta = self.clamp_joint_values(theta)

            except np.linalg.LinAlgError:
                break

        self.current_joint_angles = theta.tolist()
        return theta, error

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
