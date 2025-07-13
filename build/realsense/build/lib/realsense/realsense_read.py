import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.color_publisher = self.create_publisher(Image, '/farm/color', 10)
        self.depth_publisher = self.create_publisher(Image, '/farm/depth', 10)
        self.bridge = CvBridge()


        self.pipeline = None
        self.config = rs.config()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)    

        self.spatial_filter = rs.spatial_filter()   
        self.temporal_filter = rs.temporal_filter() 

        # === Retry settings ===
        self.connected = False
        self.retry_count = 0
        self.max_retries = 5

        # === Timer ===
        self.timer = self.create_timer(0.01, self.publish_frames)

        # Try to connect
        self.connect_camera()

    def connect_camera(self):
        self.get_logger().info("Attempting to connect to RealSense camera...")
        self.get_logger().info("1920x1080 color stream, 1280x720 depth stream, 30 FPS")
        try:
            self.pipeline = rs.pipeline()
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

            self.pipeline.start(self.config)
            self.connected = True
            self.retry_count = 0
            self.get_logger().info("Successfully connected to RealSense camera")

        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Failed to connect to RealSense: {str(e)}")
            self.retry_reconnect()

    def retry_reconnect(self):
        if self.retry_count < self.max_retries:
            wait_time = 2 ** self.retry_count
            self.get_logger().warn(f"Retrying in {wait_time} seconds... ({self.retry_count+1}/{self.max_retries})")
            time.sleep(wait_time)
            self.retry_count += 1
            self.connect_camera()
        else:
            self.get_logger().error("Max retry attempts reached. Check the camera connection.")

    def publish_frames(self):
        if not self.connected:
            self.get_logger().warn("Camera not connected. Skipping.")
            return
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                self.get_logger().warn("Invalid frames. Skipping.")
                return

            # === Apply filters ===
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)

            # === Convert to numpy ===
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # === Convert to ROS image messages ===
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')

            now = self.get_clock().now().to_msg()
            color_msg.header.stamp = now
            depth_msg.header.stamp = now

            self.color_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)

            # self.get_logger().info("Published aligned /camera/color and /camera/depth")

            # === Visualization ===
            cv2.imshow("Color Frame", color_image)
            depth_colormap = cv2.convertScaleAbs(depth_image, alpha=0.03)
            depth_colormap_color = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
            cv2.imshow("Depth Frame (Color)", depth_colormap_color)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Publishing error: {str(e)}")
            self.connected = False
            self.retry_reconnect()

    def destroy(self):
        if self.pipeline:
            self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - stopping node.")
    finally:
        node.destroy()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
