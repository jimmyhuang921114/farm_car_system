import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class RGBPixelTo3D(Node):
    def __init__(self):
        super().__init__('rgb_pixel_to_3d_node')

        # 訂閱 OCR 偵測到的文字與 pixel 座標
        self.text_sub = self.create_subscription(
            String, '/ocr_results', self.text_callback, 10)

        # 訂閱對齊後深度圖像
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10)

        # 訂閱彩色圖像（用於顯示畫面）
        self.color_sub = self.create_subscription(
            Image, '/camera/color', self.color_callback, 10)

        self.pose_pub = self.create_publisher(Pose, '/text_coordinate', 10)
        self.bridge = CvBridge()

        # 彩色相機 內參（需根據你的實機調整）
        self.fx = 919.0689
        self.fy = 919.3917
        self.cx = 623.3080
        self.cy = 369.0916

        self.detected_texts = []
        self.color_image = None

    def text_callback(self, msg):
        self.detected_texts = [item.split(',') for item in msg.data.split(';') if len(item.split(',')) == 3]

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        if self.color_image is None:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        vis_image = self.color_image.copy()

        for text, u_str, v_str in self.detected_texts:
            try:
                u, v = int(float(u_str)), int(float(v_str))
                if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
                    continue

                Z = depth_image[v, u] / 1000.0  # mm → m
                if Z <= 0 or np.isnan(Z) or np.isinf(Z):
                    continue

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                # 發布 3D 座標
                pose = Pose()
                pose.position.x = float(X)
                pose.position.y = float(Y)
                pose.position.z = float(Z)
                self.pose_pub.publish(pose)

                # 顯示在 OpenCV 畫面上
                cv2.circle(vis_image, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(vis_image, f'{text} ({X:.2f},{Y:.2f},{Z:.2f})', (u+10, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                self.get_logger().info(f'Text "{text}" at (u,v)=({u},{v}) -> (X,Y,Z)=({X:.2f}, {Y:.2f}, {Z:.2f})')

            except Exception as e:
                self.get_logger().warn(f'Error processing point: {e}')

        # 顯示視覺化結果
        cv2.imshow("Text Detection with 3D", vis_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RGBPixelTo3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
