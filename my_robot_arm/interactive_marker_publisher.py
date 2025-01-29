import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarkerFeedback

class InteractiveMarkerPublisher(Node):
    def __init__(self):
        super().__init__('interactive_marker_publisher')

        # Initialize the interactive marker server
        self.server = InteractiveMarkerServer(self, "interactive_marker_server")

        # Create and configure the interactive marker
        self.create_interactive_marker()

        # Publisher for the target position
        self.target_position_publisher = self.create_publisher(Point, '/target_position', 10)

        # Subscriber for feedback from the interactive marker
        self.feedback_subscriber = self.create_subscription(
            InteractiveMarkerFeedback,
            "/interactive_marker_server/feedback",
            self.marker_feedback_callback,
            10
        )

        self.get_logger().info("Interactive Marker Publisher Node initialized.")

        # Publish initial target position
        self.publish_initial_target_position()

    def create_interactive_marker(self):
        """
        Creates and inserts an interactive marker into the server.
        """
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = "target_marker"
        marker.description = "3D Interactive Marker (Move + Rotate)"

        # Set the initial pose
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0

        # Add control for move/rotate interaction
        control = InteractiveMarkerControl()
        control.name = "move_and_rotate_3d"
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        control.always_visible = True

        # Visualization of marker (sphere)
        visualization_marker = Marker()
        visualization_marker.type = Marker.SPHERE
        visualization_marker.scale.x = 0.05
        visualization_marker.scale.y = 0.05
        visualization_marker.scale.z = 0.05
        visualization_marker.color.r = 0.0
        visualization_marker.color.g = 0.5
        visualization_marker.color.b = 1.0
        visualization_marker.color.a = 1.0

        control.markers.append(visualization_marker)

        # Append control to the marker
        marker.controls.append(control)

        # Insert marker into the server
        self.server.insert(marker)

        # Apply changes to the server
        self.server.applyChanges()

    def marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback triggered when the interactive marker is moved in RViz.
        Publishes the updated position to the /target_position topic.
        """
        # Extract the position of the marker from the feedback
        position = feedback.pose.position

        # Log the position for debugging
        self.get_logger().info(f"Marker moved to position: x={position.x}, y={position.y}, z={position.z}")

        # Create a Point message and publish it
        target_position = Point()
        target_position.x = position.x
        target_position.y = position.y
        target_position.z = position.z
        self.target_position_publisher.publish(target_position)

    def publish_initial_target_position(self):
        """
        Publish an initial target position to initialize the interactive marker and robot model.
        """
        initial_position = Point()
        initial_position.x = 1.0
        initial_position.y = 0.2
        initial_position.z = 0.5
        self.target_position_publisher.publish(initial_position)

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerPublisher()

    # Allow for some initialization time
    node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
