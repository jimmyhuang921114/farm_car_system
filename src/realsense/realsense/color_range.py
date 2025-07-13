import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# TOPIC NAMES
COLOR_IMAGE = '/camera/color'

class ColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect')
        self.bridge = CvBridge()
        
        # 初始化滑动条值
        self.hsv_ranges = {
            "Blue": {'min': [78, 89, 24], 'max': [115, 255, 255]},
            "Yellow": {'min': [13, 104, 55], 'max': [46, 221, 255]},
            "Green": {'min': [42, 52, 30], 'max': [77, 255, 255]}
        }
        
        # 創建調整窗口
        self.create_adjust_windows()
        
        # 訂閱彩色影像
        self.create_subscription(Image, COLOR_IMAGE, self.color_callback, 10)
        self.color_image = None

        # 遮罩顯示模式 (0: 單獨窗口, 1: 合併顯示, 2: 疊加顯示)
        self.mask_mode = 2  
        self.init_windows()

    def init_windows(self):
        """初始化顯示窗口"""
        cv2.namedWindow('Color Detection', cv2.WINDOW_NORMAL)
        if self.mask_mode == 0:
            for color in self.hsv_ranges:
                cv2.namedWindow(f"{color} Mask", cv2.WINDOW_NORMAL)
        elif self.mask_mode == 1:
            cv2.namedWindow('All Masks', cv2.WINDOW_NORMAL)
        elif self.mask_mode == 2:
            cv2.namedWindow('Overlay Display', cv2.WINDOW_NORMAL)

        # 設置窗口位置
        self.arrange_windows()

    def arrange_windows(self):
        """自動排列窗口位置"""
        screen_width = 1920  # 根據實際螢幕分辨率調整
        x_offset = 0
        y_offset = 0
        
        # 調整窗口基礎位置
        cv2.moveWindow('Color Detection', x_offset, y_offset)
        x_offset += 640
        
        if self.mask_mode == 0:
            for color in self.hsv_ranges:
                cv2.moveWindow(f"{color} Mask", x_offset, y_offset)
                x_offset += 320
                if x_offset > screen_width - 320:
                    x_offset = 0
                    y_offset += 240
        elif self.mask_mode == 1:
            cv2.moveWindow('All Masks', x_offset, y_offset)
        elif self.mask_mode == 2:
            cv2.moveWindow('Overlay Display', x_offset, y_offset)

    def create_adjust_windows(self):
        """創建HSV調整窗口和滑動條"""
        for color in self.hsv_ranges:
            cv2.namedWindow(f"{color} HSV Adjust")
            cv2.createTrackbar('H Min', f"{color} HSV Adjust", self.hsv_ranges[color]['min'][0], 180, self.nothing)
            cv2.createTrackbar('S Min', f"{color} HSV Adjust", self.hsv_ranges[color]['min'][1], 255, self.nothing)
            cv2.createTrackbar('V Min', f"{color} HSV Adjust", self.hsv_ranges[color]['min'][2], 255, self.nothing)
            cv2.createTrackbar('H Max', f"{color} HSV Adjust", self.hsv_ranges[color]['max'][0], 180, self.nothing)
            cv2.createTrackbar('S Max', f"{color} HSV Adjust", self.hsv_ranges[color]['max'][1], 255, self.nothing)
            cv2.createTrackbar('V Max', f"{color} HSV Adjust", self.hsv_ranges[color]['max'][2], 255, self.nothing)

    def nothing(self, x):
        pass

    def update_hsv_values(self):
        for color in self.hsv_ranges:
            self.hsv_ranges[color]['min'][0] = cv2.getTrackbarPos('H Min', f"{color} HSV Adjust")
            self.hsv_ranges[color]['min'][1] = cv2.getTrackbarPos('S Min', f"{color} HSV Adjust")
            self.hsv_ranges[color]['min'][2] = cv2.getTrackbarPos('V Min', f"{color} HSV Adjust")
            self.hsv_ranges[color]['max'][0] = cv2.getTrackbarPos('H Max', f"{color} HSV Adjust")
            self.hsv_ranges[color]['max'][1] = cv2.getTrackbarPos('S Max', f"{color} HSV Adjust")
            self.hsv_ranges[color]['max'][2] = cv2.getTrackbarPos('V Max', f"{color} HSV Adjust")

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.update_hsv_values()
            
            # 獲取檢測結果和遮罩
            display_img, masks = self.detect_color()
            
            # 顯示調整窗口
            for color in self.hsv_ranges:
                cv2.imshow(f"{color} HSV Adjust", np.zeros((100, 500, 3), np.uint8))
            
            # 顯示遮罩
            self.show_masks(masks)
            
            cv2.imshow('Color Detection', display_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'影像處理錯誤: {str(e)}')

    def detect_color(self):
        hsv_img = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        display_img = self.color_image.copy()
        masks = {}

        for color in self.hsv_ranges:
            lower = np.array(self.hsv_ranges[color]['min'], dtype="uint8")
            upper = np.array(self.hsv_ranges[color]['max'], dtype="uint8")
            
            mask = cv2.inRange(hsv_img, lower, upper)
            masks[color] = mask
            
            # 尋找輪廓並繪製
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(display_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(display_img, f"{color}", (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return display_img, masks

    def show_masks(self, masks):
        """顯示遮罩的不同模式"""
        if self.mask_mode == 0:
            # 模式0：單獨窗口顯示每個遮罩
            for color, mask in masks.items():
                colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                cv2.imshow(f"{color} Mask", colored_mask)
                
        elif self.mask_mode == 1:
            # 模式1：合併顯示所有遮罩
            combined = np.zeros_like(self.color_image)
            colors = {'Blue': (255,0,0), 'Yellow': (0,255,255), 'Green': (0,255,0)}
            
            for color, mask in masks.items():
                colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                colored_mask = cv2.bitwise_and(colored_mask, colored_mask, mask=mask)
                colored_mask = cv2.addWeighted(combined, 1, colored_mask, 1, 0)
                combined = cv2.add(combined, colored_mask)
            
            cv2.imshow('All Masks', combined)
            
        elif self.mask_mode == 2:
            # 模式2：疊加顯示在原始影像
            overlay = self.color_image.copy()
            colors = {'Blue': (255,0,0), 'Yellow': (0,255,255), 'Green': (0,255,0)}
            
            for color, mask in masks.items():
                overlay[mask > 0] = colors[color]
            
            alpha = 0.4  # 透明度
            output = cv2.addWeighted(self.color_image, 1-alpha, overlay, alpha, 0)
            cv2.imshow('Overlay Display', output)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetect()
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