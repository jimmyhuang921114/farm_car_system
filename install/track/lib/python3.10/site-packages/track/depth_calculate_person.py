import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class PeopleDepthNode(Node):
    def __init__(self):
        super().__init__('people_depth_node')
        
        # 建立 CvBridge 以處理影像
        self.bridge = CvBridge()

        # 訂閱深度影像
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth', 
            self.depth_callback, 
            10
        )
        
        # 訂閱人物追蹤結果 (x1, y1, x2, y2)
        self.track_sub = self.create_subscription(
            Float32MultiArray, 
            '/people_track', 
            self.people_track_callback, 
            10
        )
        
        # 發布目標的深度資訊 (XZ 平面座標)
        self.position_pub = self.create_publisher(
            Float32MultiArray, 
            '/people_position', 
            10
        )
        
        # 相機內參 (請根據你的相機實際值進行調整；或從 /camera_info 訂閱來自動更新)
        self.fx = 607.0333862304688
        self.fy = 606.1414794921875  # 若要擴充到 Y 方向也可以用到 fy
        self.cx = 321.22332763671875
        self.cy = 239.4711456298828

        # 相機在機器人本體坐標系的偏移量 (單位：m)
        self.camera_offset_x = 0.0
        self.camera_offset_z = 0.4
        
        # 深度影像暫存
        self.depth_img = None

    def depth_callback(self, msg):
        """儲存深度影像 (16UC1 格式)，更新 self.depth_img"""
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"depth_callback 轉換影像失敗: {e}")

    def people_track_callback(self, msg):
        """
        接收人物邊界框資訊 (x1, y1, x2, y2)
        這邊假設只追蹤單一目標。
        """
        if len(msg.data) == 4:
            x1, y1, x2, y2 = msg.data
            # 取中心像素座標 (cx, cy)
            cx = int((x1 + x2) // 2)
            cy = int((y1 + y2) // 2)

            # 嘗試計算實際座標 (X, Z)
            result = self.pixel_to_xz(cx, cy)
            if result is not None:
                x_real, z_real = result
                self.publish_position(x_real, z_real)
            else:
                self.get_logger().warn("座標轉換失敗，無法取得有效深度值。")

        else:
            self.get_logger().warn("接收到的 bbox 座標長度不正確，預期為 4 個值。")

    def pixel_to_xz(self, x_pixel: int, y_pixel: int):
        """
        將 (x_pixel, y_pixel) 轉換成相機座標系中的 (X, Z)。
        若深度資料無效，則回傳 None。
        """
        # 確認 depth_img 是否存在
        if self.depth_img is None:
            self.get_logger().warn("尚未接收到深度影像，無法進行座標轉換。")
            return None
        
        # 檢查是否超出深度影像的有效範圍
        if (x_pixel < 0 or y_pixel < 0 or 
            x_pixel >= self.depth_img.shape[1] or 
            y_pixel >= self.depth_img.shape[0]):
            self.get_logger().warn("像素座標超出深度影像範圍")
            return None
        
        # 從深度圖取得該像素的深度值 (單位：mm)
        depth_value = self.depth_img[y_pixel, x_pixel]
        
        # 濾除深度值異常 (0 或過大)
        if depth_value == 0 or depth_value > 5000:
            self.get_logger().warn(f"無效的深度數據: {depth_value}")
            return None

        # 轉換為公尺 (m)
        depth_m = depth_value / 1000.0

        # 基於 pinhole camera model 計算實際座標 (X, Z)
        x_real = ((x_pixel - self.cx) * depth_m) / self.fx + self.camera_offset_x
        z_real = depth_m + self.camera_offset_z

        return x_real, z_real

    def publish_position(self, x_real: float, z_real: float):
        """
        將計算好的 (X, Z) 發布出去
        """
        position_msg = Float32MultiArray()
        position_msg.data = [x_real, z_real]
        self.position_pub.publish(position_msg)
        
        self.get_logger().info(f"人物位置: X={x_real:.2f} m, Z={z_real:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = PeopleDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
