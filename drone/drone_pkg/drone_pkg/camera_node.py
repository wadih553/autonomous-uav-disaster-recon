#!/usr/bin/env python3
"""
camera_node.py
----------------
Captures live video from the onboard 720p USB webcam and publishes it as a
ROS2 topic (sensor_msgs/Image) so that it can be relayed to the ground
station over ROSBridge / WebSocket, and consumed there by the YOLOv8 human
detection and fire/smoke detection models.

Hardware reference (FYP report, Ch. 3.2.3.5): 720p USB webcam @ 30 fps,
connected to the Raspberry Pi 4B via USB. Kept lightweight on purpose --
the Pi only streams frames, it does NOT run inference (that is offloaded
to the ground station server, see ground_station/server/detection.py).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - allows import on non-Pi dev machines
    cv2 = None
    CvBridge = None


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 70)

        self.device_index = self.get_parameter('device_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.raw_pub = self.create_publisher(Image, 'drone/camera/image_raw', 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'drone/camera/image_raw/compressed', 10
        )

        self.bridge = CvBridge() if CvBridge else None
        self.cap = None
        self._init_capture()

        timer_period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(timer_period, self._capture_and_publish)
        self.get_logger().info(
            f'camera_node started (device={self.device_index}, '
            f'{self.frame_width}x{self.frame_height}@{self.fps}fps)'
        )

    def _init_capture(self):
        if cv2 is None:
            self.get_logger().error('OpenCV not available - camera_node cannot capture frames.')
            return
        self.cap = cv2.VideoCapture(self.device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_index}')

    def _capture_and_publish(self):
        if self.cap is None or not self.cap.isOpened():
            return
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera read failed, skipping frame')
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        if self.publish_compressed:
            ok, buf = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if ok:
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.compressed_pub.publish(msg)
        else:
            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header = header
                self.raw_pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
