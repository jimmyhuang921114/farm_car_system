import sys
import time
import math
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QSplitter, QPlainTextEdit, QPushButton, QWidget, QDialog, QGroupBox, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QObject, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QImage, QPixmap
import subprocess
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===================== ROS信号传输类 =====================
class RosSignals(QObject):
    new_color_image = Signal(object)
    new_depth_image = Signal(object)
    new_robot_pose = Signal(object)
    new_text_message = Signal(str)
    new_color_message = Signal(str)

# ===================== 3D Matplotlib 机器人轨迹图 =====================
class Matplotlib3DPlot(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Matplotlib 3D 机器人轨迹图")
        self.setGeometry(200, 200, 600, 500)

        # ✅ Matplotlib 3D 图表
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X 坐标")
        self.ax.set_ylabel("Y 坐标")
        self.ax.set_zlabel("Z 坐标")
        self.ax.set_title("机器人轨迹")

        self.trajectory = []  # 存储轨迹点

    def update_trajectory(self, x, y, z=0):
        """ ✅ 添加新的机器人位置到轨迹，并更新 Matplotlib 3D 图 """
        self.trajectory.append((x, y, z))

        # ✅ 清除旧图
        self.ax.clear()
        self.ax.set_xlabel("X 坐标")
        self.ax.set_ylabel("Y 坐标")
        self.ax.set_zlabel("Z 坐标")
        self.ax.set_title("机器人轨迹")

        # ✅ 绘制轨迹
        if len(self.trajectory) > 1:
            x_vals, y_vals, z_vals = zip(*self.trajectory)
            self.ax.plot(x_vals, y_vals, z_vals, marker="o", linestyle="-", color="r")

        # ✅ 更新 Matplotlib 窗口
        plt.draw()
        plt.pause(0.01)

# ===================== 3D 机器人地图 (PyQtGraph OpenGL) =====================
class ThreeDMap(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("3D 机器人地图")
        self.setCameraPosition(distance=10, elevation=20, azimuth=45)

        # ✅ 添加网格
        grid = gl.GLGridItem()
        grid.setSize(10, 10)
        grid.setSpacing(1, 1)
        self.addItem(grid)

        # ✅ 机器人位置 (蓝色球)
        self.robot_marker = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), color=(0, 0, 1, 1), size=10)
        self.addItem(self.robot_marker)

        # ✅ 机器人轨迹 (红色路径)
        self.trajectory = []
        self.path_line = gl.GLLinePlotItem()
        self.addItem(self.path_line)

    @Slot(object)
    def update_robot_pose(self, pose):
        x, y, theta = pose
        self.robot_marker.setData(pos=np.array([[x, y, 0]]))

        # ✅ 记录轨迹
        self.trajectory.append([x, y, 0])
        if len(self.trajectory) > 1:
            self.path_line.setData(pos=np.array(self.trajectory), color=(1, 0, 0, 1), width=2)

# ===================== 二维地图显示组件 =====================
class EnhancedCoordinateViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scale = 50.0  # 50 像素 = 1 米
        self.robot_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.navigation_path = []
        self.setMinimumSize(800, 600)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

        # ✅ **左上角偏移参数**
        OFFSET_X = -200  # **向左偏移 200px**
        OFFSET_Y = -150  # **向上偏移 150px**

        def world_to_pixel(x, y):
            return QPointF(
                x * self.scale + self.width() / 2 + OFFSET_X,  # 左移
                self.height() / 2 - y * self.scale + OFFSET_Y  # 上移
            )

        # 画网格线
        painter.setPen(QPen(Qt.gray, 1, Qt.DotLine))
        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)

        grid_range = 3  # 以 ±3m 为边界
        for x in range(-grid_range, grid_range + 1):
            start = world_to_pixel(x, -grid_range)
            end = world_to_pixel(x, grid_range)
            painter.drawLine(start, end)
            if x % 1 == 0:
                painter.drawText(start.x() + 5, start.y() + 15, f"{x}m")

        for y in range(-grid_range, grid_range + 1):
            start = world_to_pixel(-grid_range, y)
            end = world_to_pixel(grid_range, y)
            painter.drawLine(start, end)
            if y % 1 == 0:
                painter.drawText(start.x() + 5, start.y() - 5, f"{y}m")

        # **画出机器人**
        robot_center = world_to_pixel(*self.robot_pose[:2])
        painter.setBrush(QColor(70, 130, 180))
        painter.drawEllipse(robot_center, 10, 10)

        dx = math.cos(self.robot_pose[2]) * 20
        dy = math.sin(self.robot_pose[2]) * 20
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(robot_center, robot_center + QPointF(dx, -dy))

    @Slot(object)
    def update_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()

# ===================== 影像显示组件 =====================
class ImageDisplayWidget(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setText(title + "\n等待影像输入...")
        self.setStyleSheet("""
            background: #1E1E1E;
            color: #FFFFFF;
            font: 16px;
            border: 2px solid #3E3E3E;
            border-radius: 5px;
        """)
        self.setMinimumSize(300, 200)  # 固定影像显示组件最小大小

    def update_image(self, qimg):
        if not qimg.isNull():
            self.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

# ===================== 主窗口类 =====================  
class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.bridge = CvBridge()
        self.ros_node = ros_node  # 存储 ROS 2 节点
        self.matplotlib_3d_window = Matplotlib3DPlot(self)  # ✅ Matplotlib 窗口
        self.init_ui()  # 移除註解，正確呼叫 init_ui 方法
        self.init_ros()

    def init_ui(self):  # 正確定義 init_ui 方法
        self.setWindowTitle("ROS2 机器人监控系统")
        self.setGeometry(0, 0, 1280, 720)  # 设置窗口大小
        
        # 主分割布局
        main_splitter = QSplitter(Qt.Horizontal)

        # 左侧面板
        left_panel = QSplitter(Qt.Vertical)

        self.node_status_widget = QWidget()
        self.node_status_widget.setStyleSheet("background-color: white; border-radius: 10px;")
        node_status_layout = QVBoxLayout()

        # ✅ 每个节点都有唯一键名
        self.nodes = {
            "camera_color": "彩色影像",
            "box_edge_detect": "边缘检测影像",
            "color_detect": "颜色检测影像",
            "text_detect": "文本检测影像",
            "color_order": "颜色排序指令",
            "text_detect_cmd": "文本检测指令",
        }

        self.node_lights = {}  # 存储灯号

        for node, label_text in self.nodes.items():
            row_layout = QHBoxLayout()
            
            # ✅ 创建状态灯
            light = QLabel()
            light.setFixedSize(20, 20)
            light.setStyleSheet("background-color: gray; border-radius: 10px; border: 1px solid black;")  # 默认灰色

            # ✅ 创建状态标签（名称）
            label = QLabel(label_text)
            label.setStyleSheet("font: 14px; color: black; background-color: white;")

            # ✅ 添加到布局
            row_layout.addWidget(light)  
            row_layout.addWidget(label)
            row_layout.addStretch()

            node_status_layout.addLayout(row_layout)

            # ✅ 存储灯号
            self.node_lights[node] = light

        self.node_status_widget.setLayout(node_status_layout)

        self.map_view = EnhancedCoordinateViewWidget()
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(150)

        # ✅ 4️⃣ 把「節點狀態燈號 + 日誌」放在一起
        log_container = QVBoxLayout()
        log_container.addWidget(self.node_status_widget)  # **上方的燈號**
        log_container.addWidget(self.log_view)  # **下方的系統日誌**

        log_widget = QWidget()
        log_widget.setLayout(log_container)
        left_panel.addWidget(self.map_view)
        left_panel.addWidget(self.log_view)
        
        # 右侧面板
        right_panel = QSplitter(Qt.Vertical)
        grid = QGridLayout()
        
        # 更新为四个影像显示组件
        self.cam_views = {
            'color': ImageDisplayWidget("camera_color_image"),
            'depth': ImageDisplayWidget("box_edge_detect_image"),
            'mask': ImageDisplayWidget("color_detect_image"),
            'edge': ImageDisplayWidget("text_detect_image")
        }
        
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for (key, view), pos in zip(self.cam_views.items(), positions):
            grid.addWidget(view, *pos)
        
        right_panel.setLayout(grid)
        
        main_splitter.addWidget(left_panel)
        main_splitter.addWidget(right_panel)
        self.setCentralWidget(main_splitter)

        # ✅ 設定定時器，每秒檢查節點狀態
        self.node_status_timer = QTimer(self)
        self.node_status_timer.timeout.connect(self.update_node_status)
        self.node_status_timer.start(1000)  # **每秒檢查一次**
        
    def update_node_status(self):
        """ ✅ 透过 `ros2 node list` 检测节点状态，并更新灯号颜色 """
        try:
            result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
            active_nodes = result.stdout.split("\n")  # 获取当前在线的 ROS2 节点

            for node, light in self.node_lights.items():
                if node in active_nodes:
                    light.setStyleSheet("background-color: green; border-radius: 10px; border: 1px solid black;")  # 🟢 在线
                else:
                    light.setStyleSheet("background-color: red; border-radius: 10px; border: 1px solid black;")  # 🔴 离线

        except Exception as e:
            self.log_view.appendPlainText(f"❌ 无法获取节点状态: {e}")

        # ✅ 确保定时器运行
        self.node_status_timer = QTimer(self)
        self.node_status_timer.timeout.connect(self.update_node_status)
        self.node_status_timer.start(1000)  # **每秒检查一次**

    def init_ros(self):
        self.signals = RosSignals()
        # 使用传递给构造函数的 ros_node
        self.signals.new_color_image.connect(self.update_color_image) # 确保连接到正确的方法
        self.signals.new_depth_image.connect(self.update_depth_image)
        self.signals.new_text_message.connect(self.log_text_message)
        self.signals.new_color_message.connect(self.log_color_message)
        self.signals.new_robot_pose.connect(self.map_view.update_robot_pose)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)

    @Slot(object)
    def update_color_image(self, cv_img):
        qimg = self.bridge.cv2_to_qimg(cv_img)
        self.cam_views['color'].update_image(qimg)

    @Slot(object)
    def update_depth_image(self, cv_img):
        qimg = self.bridge.cv2_to_qimg(cv_img)
        self.cam_views['depth'].update_image(qimg)

    @Slot(str)
    def log_text_message(self, message):
        self.log_view.appendPlainText(f"[文本检测]: {message}")

    @Slot(str)
    def log_color_message(self, message):
        self.log_view.appendPlainText(f"[颜色检测]: {message}")

# ===================== ROS2节点类 =====================
class Ros2Node(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.create_subscription(Image, '/camera/color', self.color_callback, 10)
        self.create_subscription(Image, 'box_edge_detect', self.box_edge_detect_callback, 10)
        self.create_subscription(Image, 'color_detect', self.color_detect_callback, 10)
        self.create_subscription(Image, 'text_detect', self.text_detect_callback, 10)
        self.create_subscription(Float32MultiArray, 'color_order', self.color_order_callback, 10)
        self.create_subscription(Float32MultiArray, 'word_detect', self.word_detect_callback, 10)
        self.create_subscription(Float32MultiArray, '/robot/pose', self.robot_pose_callback, 10)
        self.bridge = CvBridge()
        self.color_image = None
        self.box_edge_detect = None
        self.color_detect = None
        self.text_detect = None
        self.word_detect = None
        self.color_order = None

    @Slot(Image)
    def color_callback(self, msg):
        self.get_parent().signals.new_color_image.emit(self.bridge.imgmsg_to_cv2(msg))

    @Slot(Image)
    def box_edge_detect_callback(self, msg):
        self.get_parent().signals.new_depth_image.emit(self.bridge.imgmsg_to_cv2(msg))

    @Slot(Image)
    def color_detect_callback(self, msg):
        self.get_parent().signals.new_depth_image.emit(self.bridge.imgmsg_to_cv2(msg))

    @Slot(Image)
    def text_detect_callback(self, msg):
        self.get_parent().signals.new_depth_image.emit(self.bridge.imgmsg_to_cv2(msg))

    @Slot(Float32MultiArray)
    def color_order_callback(self, msg):
        pass  # 这里可以添加颜色检测相关的处理逻辑

    @Slot(Float32MultiArray)
    def word_detect_callback(self, msg):
        pass  # 这里可以添加文字检测相关的处理逻辑

    @Slot(Float32MultiArray)
    def robot_pose_callback(self, msg):
        self.get_parent().signals.new_robot_pose.emit(tuple(msg.data))

    def get_parent(self):
        return QApplication.instance().activeWindow()

# ===================== 主程序入口 =====================
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = None

    try:
        ros_node = Ros2Node() # 建立ros_node
        window = MainWindow(ros_node) # 将ros_node传入
        window.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"程序发生错误: {e}")
    finally:
        if window and hasattr(window, 'ros_node'):
            window.ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()