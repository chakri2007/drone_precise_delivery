import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')

        # Declare and read parameters
        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('dictionary', 2)
        self.declare_parameter('marker_size', 2.0)

        self.aruco_id = self.get_parameter('aruco_id').get_parameter_value().integer_value
        self.dictionary_id = self.get_parameter('dictionary').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publisher
        self.image_pub = self.create_publisher(Image, '/image_proc', 10)

        self.get_logger().info('ArucoTracker Python node started')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('Camera info not yet received')
            return

        # Convert image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ArUco detection
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if ids is not None:
            self.get_logger().info(f'Detected ArUco IDs: {ids.flatten()}')

            # Draw all detected markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate and draw pose for specified ID
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.aruco_id:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    for rvec, tvec in zip(rvecs, tvecs):
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        else:
            self.get_logger().info('No markers detected')

        # Publish processed image
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(out_msg)

        # Show debug image window
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
