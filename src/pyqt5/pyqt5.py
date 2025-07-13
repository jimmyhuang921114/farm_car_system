import sys
import cv2
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

class VideoStream(QWidget):
    def __init__(self, camera_id=0, parent=None):
        super(VideoStream, self).__init__(parent)
        self.camera_id = camera_id
        self.label = QLabel(self)
        self.cap = cv2.VideoCapture(self.camera_id)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms 更新一次
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width, channel = frame.shape
            bytes_per_line = channel * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.label.setPixmap(QPixmap.fromImage(q_img))

    def closeEvent(self, event):
        self.cap.release()

class ThreeDViewer(QWidget):
    def __init__(self, parent=None):
        super(ThreeDViewer, self).__init__(parent)
        self.label = QLabel("三維點雲顯示")
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__('ros2_text_subscriber')
        self.subscribers = []
        self.text_widgets = []
        
    def add_subscription(self, topic, text_widget):
        self.text_widgets.append(text_widget)
        sub = self.create_subscription(
            String,
            topic,
            lambda msg, widget=text_widget: widget.setPlainText(msg.data),
            10
        )
        self.subscribers.append(sub)

class StatusLight(QLabel):
    def __init__(self, parent=None):
        super(StatusLight, self).__init__(parent)
        self.setFixedSize(50, 50)
        self.setStyleSheet("background-color: red; border-radius: 25px;")

    def update_status(self, status):
        color = "green" if status else "red"
        self.setStyleSheet(f"background-color: {color}; border-radius: 25px;")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt5 + ROS2 GUI")
        self.setGeometry(100, 100, 1200, 800)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        layout = QHBoxLayout()
        
        # 左側 - 三維圖 + 文字框
        left_layout = QVBoxLayout()
        self.three_d_viewer = ThreeDViewer()
        left_layout.addWidget(self.three_d_viewer)
        
        # ROS2 文字框
        self.text_boxes = []
        for _ in range(4):
            text_box = QTextEdit()
            text_box.setFixedHeight(50)
            left_layout.addWidget(text_box)
            self.text_boxes.append(text_box)
        
        # 右側 - 影片流
        video_layout = QGridLayout()
        self.videos = [VideoStream(i) for i in range(4)]
        video_layout.addWidget(self.videos[0], 0, 0)
        video_layout.addWidget(self.videos[1], 0, 1)
        video_layout.addWidget(self.videos[2], 1, 0)
        video_layout.addWidget(self.videos[3], 1, 1)
        
        # 狀態燈
        self.status_light = StatusLight()
        left_layout.addWidget(self.status_light, alignment=Qt.AlignCenter)
        
        layout.addLayout(left_layout, 1)
        layout.addLayout(video_layout, 2)
        
        self.central_widget.setLayout(layout)

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
