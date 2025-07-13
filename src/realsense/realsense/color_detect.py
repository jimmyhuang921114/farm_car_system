# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from std_msgs.msg import Float32MultiArray

# # TOPIC NAMES
# #SUB
# COLOR_IMAGE = '/camera/color'
# DEPTH_IMAGE = '/camera/depth'
# WORD_POSITION = 'word_order_detect'
# #PUB
# BOX_EDGE_IMAGE = 'box_edge_iamge'
# BOX_EDGE_POSITION = 'box_edge_position'

# # 黑色範圍的 HSV 值
# BLACK_HSV_RANGE = ((0, 0, 0), (180, 255, 46))

# class BoxEdgeDetect(Node):
#     def __init__(self):
#         super().__init__('box_edge_detect')
#         self.bridge = CvBridge()

#         # 訂閱彩色影像與深度影像
#         self.create_subscription(Image, COLOR_IMAGE, self.color_callback, 10)
#         self.create_subscription(Image, DEPTH_IMAGE, self.depth_callback, 10)
#         self.create_subscription(Float32MultiArray, WORD_POSITION, self.text_mid_position_callback, 10)

#         # 發布檢測結果
#         self.box_edge_image_pub = self.create_publisher(Image, BOX_EDGE_IMAGE, 10)
#         self.box_position_detect_pub = self.create_publisher(Float32MultiArray, BOX_EDGE_POSITION, 10)
        
#         # 初始化變數
#         self.color_image = None
#         self.depth_image = None
#         self.word_middle_position = None
    
#     def color_callback(self, msg):
#         """接收並處理彩色影像：偵測輪廓並發布結果。"""
#         self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         if self.depth_image is None:
#             self.get_logger().error('Depth image not received yet')
#             return
#         else:
#             self.get_logger().info('Color image received')
#             self.get_box_position()

#     def depth_callback(self, msg):
#         """接收深度影像"""
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         if self.depth_image is None:
#             self.get_logger().error('Depth image not received yet')
#             return
#         else:
#             self.get_logger().info('Depth image received')

#     def text_mid_position_callback(self, msg):
#         """接收目標文字中點位置"""
#         self.word_middle_position = msg.data
#         self.get_logger().info(f'Received word middle position: {self.word_middle_position}')

#     def get_box_position(self):
#         """檢測影像中的輪廓並顯示於 OpenCV 視窗"""
#         if self.color_image is None:
#             self.get_logger().error('No color image available')
#             return

#         # 轉換成灰階影像
#         gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)

#         # 進行二值化處理
#         ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

#         # 偵測輪廓
#         contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # 在影像上標記輪廓
#         result_image = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
#         cv2.drawContours(result_image, contours, -1, (0, 0, 255), 2)
#         self.get_logger().info(f'Detected {len(contours)} contours')

#         # 顯示二值化影像
#         cv2.imshow('Box Edge Detection - Binary', thresh)
#         cv2.imshow('Box Edge Detection - Contours', result_image)
#         cv2.waitKey(1)

#         # 發布處理後的影像
#         processed_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
#         self.box_edge_detect_pub.publish(processed_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = BoxEdgeDetect()
#     rclpy.spin(node)
#     node.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

# 訂閱話題
COLOR_IMAGE = '/camera/color'
DEPTH_IMAGE = '/camera/depth'
WORD_POSITION = 'word_order_detect'

# 發布話題
BOX_EDGE_IMAGE = 'box_edge_image'
BOX_EDGE_POSITION = 'box_edge_position'
BOX_EDGE_ORIENTATION = 'box_edge_orientation'  # 新增發佈方塊方向的話題

# 黑色範圍的 HSV 值
BLACK_HSV_RANGE = ((0, 0, 0), (180, 255, 46))

class BoxEdgeDetect(Node):
    def __init__(self):
        super().__init__('box_edge_detect')
        self.bridge = CvBridge()

        # 訂閱彩色影像、深度影像與文字點位置
        self.create_subscription(Image, COLOR_IMAGE, self.color_callback, 10)
        self.create_subscription(Image, DEPTH_IMAGE, self.depth_callback, 10)
        self.create_subscription(Float32MultiArray, WORD_POSITION, self.text_mid_position_callback, 10)

        # 發布檢測結果
        self.box_edge_image_pub = self.create_publisher(Image, BOX_EDGE_IMAGE, 10)
        self.box_position_detect_pub = self.create_publisher(Float32MultiArray, BOX_EDGE_POSITION, 10)
        self.box_orientation_pub = self.create_publisher(Float32MultiArray, BOX_EDGE_ORIENTATION, 10)  # 新增方向發佈
        
        # 初始化變數
        self.color_image = None
        self.depth_image = None
        self.word_middle_position = None

    def color_callback(self, msg):
        """接收並處理彩色影像"""
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.depth_image is not None and self.word_middle_position is not None:
            self.detect_box_edges()

    def depth_callback(self, msg):
        """接收深度影像"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def text_mid_position_callback(self, msg):
        """接收目標文字中點位置"""
        self.word_middle_position = np.array(msg.data).reshape(-1, 2)  # 轉換為 (x, z) 陣列
        self.get_logger().info(f'Received word middle position: {self.word_middle_position}')

    def detect_box_edges(self):
        """檢測影像中的黑色區域來確定木頭方塊位置"""
        if self.color_image is None:
            self.get_logger().error('No color image available')
            return

        # 轉換成 HSV 影像
        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, np.array(BLACK_HSV_RANGE[0]), np.array(BLACK_HSV_RANGE[1]))

        # 偵測輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result_image = self.color_image.copy()
        box_positions = []
        box_orientations = []

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 忽略小區域
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.intp(box)  # 轉換為整數座標
                cv2.drawContours(result_image, [box], 0, (0, 255, 0), 2)

                # 獲取中心點與角度
                center_x, center_y = int(rect[0][0]), int(rect[0][1])
                angle = rect[2]  # 旋轉角度
                
                # 調整角度範圍到 -90 到 90 度
                if angle < -45:
                    angle += 90
                
                # 比較與文字點的關係來確保方向
                nearest_text_point = min(self.word_middle_position, key=lambda p: np.linalg.norm(np.array([p[0], p[1]]) - np.array([center_x, center_y])))

                # 判斷方向是否正確 (假設文字應該靠近方塊的某個固定邊)
                dx = nearest_text_point[0] - center_x
                dz = nearest_text_point[1] - center_y
                if abs(dx) > abs(dz):
                    orientation = "Horizontal" if angle < 10 else "Rotated"
                else:
                    orientation = "Vertical" if angle < 10 else "Rotated"

                self.get_logger().info(f'Box at ({center_x}, {center_y}) - Angle: {angle:.2f}, Orientation: {orientation}')

                # 存儲結果
                box_positions.append((center_x, center_y))
                box_orientations.append((center_x, center_y, angle, 1 if orientation == "Horizontal" else 0))

        # 顯示影像
        cv2.imshow('Box Edge Detection', result_image)
        cv2.imshow('Box Edge Mask', mask)
        cv2.waitKey(1)

        # 發布處理後的影像
        processed_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
        self.box_edge_image_pub.publish(processed_msg)

        # 發布木頭方塊位置
        position_msg = Float32MultiArray()
        for pos in box_positions:
            position_msg.data.extend([float(pos[0]), float(pos[1])])
        self.box_position_detect_pub.publish(position_msg)

        # 發布木頭方塊方向
        orientation_msg = Float32MultiArray()
        for orient in box_orientations:
            orientation_msg.data.extend([float(orient[0]), float(orient[1]), float(orient[2]), float(orient[3])])
        self.box_orientation_pub.publish(orientation_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoxEdgeDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()