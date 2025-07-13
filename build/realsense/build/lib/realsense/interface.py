import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget,QPushButton,QStackedWidget
from PyQt6.QtCore import Qt,QTimer  # PyQt6 需要单独导入 Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PyQt6 主窗口")
        self.setGeometry(100, 100, 800, 600)

        self.stacked_widget = QStackedWidget()

        #page 1
        self.page_ros_status = QWidget()
        self.ros_label_QLabel = QLabel("ROS2 状态", self.page_ros_status)
        self.ros_label_QLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout1 = QVBoxLayout()
        layout1.addWidget(self.ros_label_QLabel)
        self.page_ros_status.setLayout(layout1)

        #page 2
        self.page_camera = QWidget()
        self.camera_label_QLabel = QLabel("camera status", self.page_camera)
        self.camera_label_QLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout2 = QVBoxLayout()
        layout2.addWidget(self.camera_label_QLabel)
        self.page_camera.setLayout(layout2)

        #page 3
        self.page_logs =QWidget()
        self.log_label =QLabel("logs",self.page_logs)
        self.log_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout3 = QVBoxLayout()
        layout3.addWidget(self.log_label)
        self.page_logs.setLayout(layout3)

        #add the pages to the stacked widget
        self.stacked_widget.addWidget(self.page_ros_status)
        self.stacked_widget.addWidget(self.page_camera)
        self.stacked_widget.addWidget(self.page_logs)

        #create switch button
        self.switch_ros_status = QPushButton("ros2 state", self)
        self.switch_camera = QPushButton("camera_state", self)
        self.switch_logs = QPushButton("logs", self)        

        self.switch_ros_status.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))
        self.switch_camera.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        self.switch_logs.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.switch_ros_status)
        button_layout.addWidget(self.switch_camera)
        button_layout.addWidget(self.switch_logs)
        #button on the top
        button_layout.addStretch()

        #create container on the left
        button_container = QWidget()
        button_container.setLayout(button_layout)
        button_container.setFixedWidth(200)

        main_layout = QVBoxLayout()
        main_layout.addWidget(button_container)
        main_layout.addWidget(self.stacked_widget)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 创建主界面组件
        self.label = QLabel("欢迎使用 PyQt6", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # PyQt6 需要用 AlignmentFlag

        # self.open_window_button = QPushButton("Open Second Window", self)
        # self.open_window_button.clicked.connect(self.open_secondary_window)

        # layout = QVBoxLayout()
        # layout.addWidget(self.label)
        # layout.addWidget(self.open_window_button)

        # self.secondary_window = SecondWindow()
        # self.timer=QTimer()
        # self.timer.timeout.connect(self.update_ui)
        # self.timer.start(1000)

    #user interface
    def update_ui(self):
        """ ✅ 更新主界面 UI """
        if self.ros_node.received_message:
            self.label.setText(f"当前 ROS2 消息: {self.ros_node.received_message}")

            # ✅ 同步更新外挂窗口
            if self.secondary_window.isVisible():
                self.secondary_window.update_data(self.ros_node.received_message)

    def open_secondary_window(self):
        """ ✅ 点击按钮后打开独立窗口 """
        self.secondary_window.show()

class SecondWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 第二窗口")
        self.setGeometry(200, 200, 400, 300)    
        
        self.label = QLabel("这是第二窗口", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
      
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)
    #update data
    def update_data(self,data):
        self.label.setText(data)




#------Ros2 setup--------#
class VisualView(Node):
    def __init__(self):
        super().__init__("visual_view")
        self.bridge = CvBridge()
        self.color_img_sub = self.create_subscription(Image, 'camera/color', self.color_image_callback, 10)
        self.text_detect_image_sub = self.create_subscription(Image, 'text_detect_image', self.text_detect_image_callback, 10)
        self.box_edge_depth_sub = self.create_subscription(Image, 'box_edge_depth', self.box_edge_depth_callback, 10)
        self.color_detect_sub = self.create_subscription(Image, 'color_detect', self.color_detect_callback, 10)
        
        self.text_detect_sub = self.create_subscription(Float32MultiArray, 'text_detect', self.text_detect_callback, 10)
        self.box_center_position_sub = self.create_subscription(Float32MultiArray, 'box_center_position', self.box_center_position_callback, 10)
        self.color_order_sub = self.create_subscription(Float32MultiArray, 'color_order', self.color_order_callback, 10)
        self.get_logger().info("Visual View Node has been started")



        def color_image_callback(self, msg):
            self.camera_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        def text_detect_image_callback(self, msg):
            self.text_detect_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        def box_edge_depth_callback(self, msg):
            self.box_edge_depth = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        def color_detect_callback(self, msg):
            self.color_detect = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        def text_detect_callback(self, msg):
            self.text_detect = msg.data
        def box_center_position_callback(self, msg):
            self.box_center_position = msg.data
        def color_order_callback(self, msg):
            self.color_order = msg.data



        self.bridge = CvBridge()
        #status
        self.text_detect = False
        self.box_center_position = False
        self.color_order = False

        self.camera_color = False
        self.text_detect_image = False
        self.box_edge_depth = False
        self.color_detect = False



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
