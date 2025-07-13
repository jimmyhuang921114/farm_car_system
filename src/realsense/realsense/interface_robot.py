import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QSplitter, QLabel, QHBoxLayout, QVBoxLayout, QWidget, QPushButton
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 設置主窗口標題和大小
        self.setWindowTitle("Robot Interface")
        self.setGeometry(100, 100, 1280, 720)

        # 主界面布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        # 主布局（左右分割）
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)

        # 左側窗口（4 層）
        left_widget = QWidget()
        left_layout = QHBoxLayout()
        left_layout.setAlignment(Qt.AlignmentFlag.AlignRight)
        left_widget.setLayout(left_layout)

        # 左側 4 層窗口
        four_screens = QWidget()
        four_layout = QVBoxLayout()
        four_screens.setLayout(four_layout)
        four_screens.setFixedSize(640, 720)

        # 創建四個 QLabel 作為左側的四個窗口
        label1 = self.create_label("Text Detect", "lightblue")
        label2 = self.create_label("Box Center Position", "lightgreen")
        label3 = self.create_label("Color Order", "lightcoral")
        label4 = self.create_label("Camera Color", "lightyellow")

        # 將四個 QLabel 添加到左側布局
        four_layout.addWidget(label1)
        four_layout.addWidget(label2)
        four_layout.addWidget(label3)
        four_layout.addWidget(label4)

        left_layout.addWidget(four_screens)
        main_layout.addWidget(left_widget)

        # 右側控制面板
        right_widget = QWidget()
        right_layout = QVBoxLayout()
        right_widget.setLayout(right_layout)

        # 創建一個 QLabel 顯示控制面板標題
        self.label = QLabel("PyQt6 Control Panel", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # 創建一個按鈕用於打開第二個窗口
        self.open_window_button = QPushButton("Open Secondary Window", self)
        self.open_window_button.clicked.connect(self.open_secondary_window)

        # 創建一個按鈕用於退出程序
        self.exit_button = QPushButton("Exit Program", self)
        self.exit_button.clicked.connect(QApplication.instance().quit)  # 退出程序

        # 創建一個 QLabel 用於顯示 camera/color 影像
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setFixedSize(640, 480)
        right_layout.addWidget(self.video_label)

        # 將控制面板的組件添加到右側布局
        right_layout.addWidget(self.label)
        right_layout.addWidget(self.open_window_button)
        right_layout.addWidget(self.exit_button)  # 添加退出按鈕

        main_layout.addWidget(right_widget)

        # 初始化第二個窗口
        self.secondary_window = SecondWindow()

        # 初始化 ROS2 節點
        self.ros_node = VisualView()
        self.ros_node.gui = self  # 將 GUI 傳遞給 ROS2 節點

        # 定時器用於更新 UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # 每 100ms 更新一次

        # ROS2 定時器
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)  # 每 50ms 調用一次 spin_once

    def update_video(self, frame):
        """更新 QLabel 影像窗口"""
        if frame is None or frame.size == 0:
            print("Received empty frame")
            return
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
        self.video_label.setPixmap(QPixmap.fromImage(q_image))

    def create_label(self, text, color):
        """創建 QLabel 作為窗口顯示"""
        label = QLabel(text)
        label.setStyleSheet(f"background-color: {color}; font-size: 18px; border: 2px solid black;")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setFixedSize(640, 180)
        return label

    def update_ui(self):
        """更新主界面 UI"""
        if hasattr(self.ros_node, "text_detect"):
            self.label.setText(f"Current Detection: {self.ros_node.text_detect}")

            # 同步更新第二個窗口
            if self.secondary_window.isVisible():
                self.secondary_window.update_status("text_detect", self.ros_node.text_detect is not None)

    def open_secondary_window(self):
        """點擊按鈕後打開第二窗口"""
        self.secondary_window.show()

class SecondWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Secondary Window")
        self.setGeometry(200, 200, 400, 300)

        # 主布局
        self.main_layout = QVBoxLayout()
        self.label = QLabel("Status Indicators", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.label)

        # 創建多個狀態指示燈
        self.status_labels = {
            "text_detect": self.create_status_label("Text Detect: "),
            "box_center_position": self.create_status_label("Box Center Position: "),
            "color_order": self.create_status_label("Color Order: "),
            "camera_color": self.create_status_label("Camera Color: "),
            "text_detect_image": self.create_status_label("Text Detect Image: "),
            "box_edge_depth": self.create_status_label("Box Edge Depth: "),
            "color_detect": self.create_status_label("Color Detect: "),
        }
        for status_layout, _ in self.status_labels.values():
            self.main_layout.addLayout(status_layout)

        self.setLayout(self.main_layout)

    def create_status_label(self, name):
        """創建狀態指示燈"""
        layout = QHBoxLayout()

        # 狀態標籤
        label = QLabel(name)
        label.setFixedSize(200, 20)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

        # 狀態燈
        status_light = QLabel()
        status_light.setFixedSize(20, 20)
        status_light.setStyleSheet("background-color: red; border-radius: 10px;")
        layout.addWidget(status_light)
        return layout, status_light

    def update_status(self, status_name, active):
        """更新狀態指示燈顏色"""
        if status_name in self.status_labels:
            _, status_light = self.status_labels[status_name]
            if active:
                status_light.setStyleSheet("background-color: green; border-radius: 10px;")  # 綠燈
            else:
                status_light.setStyleSheet("background-color: red; border-radius: 10px;")  # 紅燈

class VisualView(Node):
    def __init__(self):
        super().__init__("visual_view")

        # 訂閱 /camera/color 主題
        self.color_img_sub = self.create_subscription(Image, '/camera/color', self.color_image_callback, 10)
        self.get_logger().info("Visual View Node has been started")

        self.bridge = CvBridge()
        self.camera_color = False

    def color_image_callback(self, msg):
        try:
            # 轉換 ROS2 影像為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")
            self.gui.update_video(frame)  # 傳遞給 GUI
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")

if __name__ == "__main__":
    rclpy.init()  # 先初始化 ROS2

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # 確保 PyQt 和 ROS2 都能運行
    sys.exit(app.exec())

    rclpy.shutdown()  # 退出時關閉 ROS2