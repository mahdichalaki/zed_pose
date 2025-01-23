import rclpy
from rclpy.node import Node
import pyzed.sl as sl
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray  # Import Float32MultiArray


class ZEDPoseNode(Node):
    def __init__(self):
        super().__init__('zed_pose_node')

        # Declare parameters for body format and camera side
        self.declare_parameter('body_format', 'BODY_18')  # Default to BODY_18
        self.declare_parameter('camera_side', 'left')  # Default to left camera
        self.declare_parameter('publish_frequency', 10.0)  # Default to 10 Hz

        # Get parameters
        body_format_param = self.get_parameter('body_format').get_parameter_value().string_value
        camera_side_param = self.get_parameter('camera_side').get_parameter_value().string_value
        publish_frequency_param = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Set body format based on parameter
        if body_format_param == 'BODY_18':
            self.body_format = sl.BODY_FORMAT.BODY_18
        elif body_format_param == 'BODY_34':
            self.body_format = sl.BODY_FORMAT.BODY_34
        elif body_format_param == 'BODY_38':
            self.body_format = sl.BODY_FORMAT.BODY_38
        else:
            self.get_logger().warn("Invalid body format provided. Defaulting to BODY_18.")
            self.body_format = sl.BODY_FORMAT.BODY_18
        
        # Set camera side based on parameter
        if camera_side_param == 'left':
            self.camera_view = sl.VIEW.LEFT
        elif camera_side_param == 'right':
            self.camera_view = sl.VIEW.RIGHT
        else:
            self.get_logger().warn("Invalid camera side provided. Defaulting to left.")
            self.camera_view = sl.VIEW.LEFT

        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.sdk_verbose = 1

        # Open the camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Error opening ZED camera: {err}")
            rclpy.shutdown()
            return

        # Define Body Tracking parameters
        self.body_params = sl.BodyTrackingParameters()
        self.body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        self.body_params.enable_tracking = True
        self.body_params.enable_segmentation = False
        self.body_params.enable_body_fitting = True
        self.body_params.body_format = self.body_format  # Use selected body format

        # Enable Positional Tracking
        positional_tracking_param = sl.PositionalTrackingParameters()
        positional_tracking_param.set_floor_as_origin = True
        self.zed.enable_positional_tracking(positional_tracking_param)

        # Enable Body Tracking module
        err = self.zed.enable_body_tracking(self.body_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Enable Body Tracking: {repr(err)}. Exit program.")
            self.zed.close()
            rclpy.shutdown()
            return

        # Set up body tracking runtime parameters
        self.body_runtime_param = sl.BodyTrackingRuntimeParameters()
        self.body_runtime_param.detection_confidence_threshold = 40

        # Create OpenCV bridge
        self.bridge = CvBridge()

        # Set up ROS2 publisher for the selected camera
        self.camera_publisher_ = self.create_publisher(Image, f'zed_pose/{camera_side_param}_image', 10)

        # Publisher for all angles
        self.angles_publisher = self.create_publisher(Float32MultiArray, 'angles', 10)

        # Calculate the timer interval based on the desired frequency
        timer_interval = 1.0 / publish_frequency_param  # Interval in seconds

        # Timer to capture and publish data
        self.timer = self.create_timer(timer_interval, self.timer_callback)

    def get_body_parts_enum(self):
        """Get the correct body parts enumeration based on the selected body format."""
        if self.body_format == sl.BODY_FORMAT.BODY_18:
            return sl.BODY_18_PARTS
        elif self.body_format == sl.BODY_FORMAT.BODY_34:
            return sl.BODY_34_PARTS
        elif self.body_format == sl.BODY_FORMAT.BODY_38:
            return sl.BODY_38_PARTS
        else:
            self.get_logger().warn("Invalid body format detected. Defaulting to BODY_18 parts.")
            return sl.BODY_18_PARTS

    def calculate_2d_angle(self, a, b, c):
        """Calculate the angle between three points (a, b, c) using the cosine rule in 2D."""
        ab = np.array([a[0] - b[0], a[1] - b[1]])
        bc = np.array([c[0] - b[0], c[1] - b[1]])

        # Calculate dot product and magnitudes
        dot_product = np.dot(ab, bc)
        magnitude_ab = np.linalg.norm(ab)
        magnitude_bc = np.linalg.norm(bc)

        # Calculate the angle in radians and convert to degrees
        if magnitude_ab * magnitude_bc == 0:  # Avoid division by zero
            return 0.0

        cos_angle = dot_product / (magnitude_ab * magnitude_bc)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip to handle numerical errors
        return np.degrees(angle)

    def calculate_3d_angle(self, a, b, c):
        """Calculate the angle between three points (a, b, c) in 3D using the cosine rule."""
        ab = np.array([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
        bc = np.array([c[0] - b[0], c[1] - b[1], c[2] - b[2]])

        # Calculate dot product and magnitudes
        dot_product = np.dot(ab, bc)
        magnitude_ab = np.linalg.norm(ab)
        magnitude_bc = np.linalg.norm(bc)

        # Calculate the angle in radians and convert to degrees
        if magnitude_ab * magnitude_bc == 0:  # Avoid division by zero
            return 0.0

        cos_angle = dot_product / (magnitude_ab * magnitude_bc)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip to handle numerical errors
        return np.degrees(angle)

    def timer_callback(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Process camera data based on the selected side
            bodies = sl.Bodies()
            self.zed.retrieve_bodies(bodies, self.body_runtime_param)

            if bodies.is_new:
                body_array = bodies.body_list
                self.get_logger().info(f"{len(body_array)} Person(s) detected")

                # Retrieve the image frame based on the camera side parameter
                image = sl.Mat()
                self.zed.retrieve_image(image, self.camera_view)
                frame = image.get_data()

                # Convert image from BGRA to BGR format
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # Initialize an empty list for angles
                angles_list = []

                # Check if there are detected bodies
                if len(body_array) > 0:
                    # Get the correct body parts enumeration
                    body_parts_enum = self.get_body_parts_enum()

                    # Draw detected bodies on the frame
                    for body in body_array:
                        keypoints_2d = body.keypoint_2d  # Use 2D keypoints for visualization
                        keypoints_3d = body.keypoint  # Use 3D keypoints for 3D angle calculation

                        # Calculate angles for the right elbow in 3D
                        right_elbow_angle = self.calculate_3d_angle(keypoints_3d[body_parts_enum.RIGHT_WRIST.value], 
                                                                    keypoints_3d[body_parts_enum.RIGHT_ELBOW.value], 
                                                                    keypoints_3d[body_parts_enum.RIGHT_SHOULDER.value])

                        # Calculate angles for the right torso in 2D
                        right_torso_angle = self.calculate_2d_angle(keypoints_2d[body_parts_enum.RIGHT_ELBOW.value], 
                                                                    keypoints_2d[body_parts_enum.RIGHT_SHOULDER.value], 
                                                                    keypoints_2d[body_parts_enum.RIGHT_HIP.value])

                        # Calculate angles for the left elbow in 3D
                        left_elbow_angle = self.calculate_3d_angle(keypoints_3d[body_parts_enum.LEFT_WRIST.value], 
                                                                keypoints_3d[body_parts_enum.LEFT_ELBOW.value], 
                                                                keypoints_3d[body_parts_enum.LEFT_SHOULDER.value])

                        # Calculate angles for the left torso in 2D
                        left_torso_angle = self.calculate_2d_angle(keypoints_2d[body_parts_enum.LEFT_ELBOW.value], 
                                                                keypoints_2d[body_parts_enum.LEFT_SHOULDER.value], 
                                                                keypoints_2d[body_parts_enum.LEFT_HIP.value])

                        # Display angles on the frame
                        #cv2.putText(frame_bgr, f"Right Elbow: {right_elbow_angle:.2f} deg", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        cv2.putText(frame_bgr, f"Right Shoulder Abduction angle: {right_torso_angle:.2f} deg", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        #cv2.putText(frame_bgr, f"Left Elbow: {left_elbow_angle:.2f} deg", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        #cv2.putText(frame_bgr, f"Left Shoulder Abduction angle: {left_torso_angle:.2f} deg", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                        # Append the calculated angles to the list
                        angles_list = [right_elbow_angle, right_torso_angle, left_elbow_angle, left_torso_angle]

                        # Draw 2D keypoints on the image (green dots)
                        for kp_2d in keypoints_2d:
                            if not np.isnan(kp_2d[0]) and not np.isnan(kp_2d[1]):
                                cv2.circle(frame_bgr, (int(kp_2d[0]), int(kp_2d[1])), 5, (0, 255, 0), -1)
                else:
                    # No person detected, set all angles to 0
                    angles_list = [0.0, 0.0, 0.0, 0.0]

                # Publish all angles in a single message
                angles_msg = Float32MultiArray()
                angles_msg.data = angles_list
                self.angles_publisher.publish(angles_msg)

                # Display the image using OpenCV
                cv2.imshow(f"ZED 2i {self.get_parameter('camera_side').get_parameter_value().string_value.capitalize()} Camera", frame_bgr)
                cv2.waitKey(1)

                # Convert OpenCV image to ROS2 Image message and publish
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                    self.camera_publisher_.publish(ros_image)
                except CvBridgeError as e:
                    self.get_logger().error(f"CV Bridge Error: {e}")


    def destroy(self):
        # Disable the body tracking module and close the camera
        self.zed.disable_body_tracking()
        self.zed.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDPoseNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

