import os
import csv
import datetime
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

# Message types
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, Float32, Int16MultiArray, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# PySide6 GUI components
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel, QTextEdit

class RecorderNode(Node):
    def __init__(self, folder_path):
        super().__init__('recorder_node')
        self.folder_path = folder_path
        self.bridge = CvBridge()
        # Open CSV file to record non-image data
        self.csv_file = open(os.path.join(folder_path, 'data.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time", "Topic", "Data"])
        
        # Dictionary for storing VideoWriter objects for images
        self.video_writers = {}
        
        # Create subscribers
        self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        self.create_subscription(Float32MultiArray, '/obstacle/xy_list', self.obstacle_callback, 10)
        self.create_subscription(Image, '/camera/color', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Image, '/tracking_image', self.tracking_callback, 10)
        self.create_subscription(Image, '/masked_image', self.masked_callback, 10)
        self.create_subscription(Int16MultiArray, '/ultrasonic_data_1', self.ultrasonic_data_1_callback, 10)
        self.create_subscription(Int16MultiArray, '/ultrasonic_data_2', self.ultrasonic_data_2_callback, 10)
        
        # Create publishers (example publishers; adjust as needed)
        self.uwb_raw_data_0_pub = self.create_publisher(Float32, 'uwb_raw_data_0', 10)
        self.uwb_raw_data_2_pub = self.create_publisher(Float32, 'uwb_raw_data_2', 10)
        self.uwb_raw_data_3_pub = self.create_publisher(Float32, 'uwb_raw_data_3', 10)
        self.uwb_raw_data_5_pub = self.create_publisher(Float32, 'uwb_raw_data_5', 10)
        
        # Create a new publisher for /people_track
        self.people_track_pub = self.create_publisher(Int32MultiArray, '/people_track', 10)
        # Set up a timer callback to publish /people_track data every second
        self.create_timer(1.0, self.people_track_callback)

    def record_csv(self, topic, data):
        """Record non-image data to CSV with time, topic, and data."""
        now = datetime.datetime.now().isoformat()
        self.csv_writer.writerow([now, topic, str(data)])
        self.csv_file.flush()

    def record_image(self, topic, cv_image):
        """
        Write image data to an AVI file:
          - Automatically create a VideoWriter on first image (using image size)
          - Write images at 30 fps
        """
        if topic not in self.video_writers:
            h, w = cv_image.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            video_filename = topic.replace('/', '') + '.avi'
            video_path = os.path.join(self.folder_path, video_filename)
            self.video_writers[topic] = cv2.VideoWriter(video_path, fourcc, 30, (w, h))
        self.video_writers[topic].write(cv_image)

    def uwb_callback(self, msg: PointStamped):
        self.record_csv('/uwb_filtered_position', msg.point)

    def obstacle_callback(self, msg: Float32MultiArray):
        self.record_csv('/obstacle/xy_list', msg.data)

    def rgb_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.record_image('/camera/color', cv_image)
        except Exception as e:
            self.get_logger().error("Error converting RGB image: " + str(e))

    def depth_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            cv_image = cv2.convertScaleAbs(cv_image, alpha=0.03)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            self.record_image('/camera/depth', cv_image)
        except Exception as e:
            self.get_logger().error("Error converting depth image: " + str(e))
    
    def tracking_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.record_image('/tracking_image', cv_image)
        except Exception as e:
            self.get_logger().error("Error converting tracking image: " + str(e))

    def masked_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.record_image('/masked_image', cv_image)
        except Exception as e:
            self.get_logger().error("Error converting masked image: " + str(e))

    def ultrasonic_data_1_callback(self, msg: Int16MultiArray):
        self.record_csv('/ultrasonic_data_1', msg.data)

    def ultrasonic_data_2_callback(self, msg: Int16MultiArray):
        self.record_csv('/ultrasonic_data_2', msg.data)
        
    def people_track_callback(self):
        """
        Periodically publish dummy people tracking data and record it.
        Replace the dummy data with actual tracking information as needed.
        """
        msg = Int32MultiArray()
        # Dummy data: list of tracked person IDs (example)
        msg.data = [1, 2, 3]
        self.people_track_pub.publish(msg)
        self.record_csv('/people_track', msg.data)

    def destroy_node(self):
        # Release VideoWriter objects and close CSV file before shutdown
        for vw in self.video_writers.values():
            vw.release()
        self.csv_file.close()
        super().destroy_node()

class RecorderGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Recorder GUI")
        self.layout = QVBoxLayout()
        
        self.folder_label = QLabel("Enter folder name:")
        self.layout.addWidget(self.folder_label)
        
        self.folder_line_edit = QLineEdit()
        self.layout.addWidget(self.folder_line_edit)
        
        self.start_button = QPushButton("Start Recording")
        self.start_button.clicked.connect(self.start_recording)
        self.layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Recording")
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False)
        self.layout.addWidget(self.stop_button)
        
        self.status_label = QLabel("Status: Idle")
        self.layout.addWidget(self.status_label)

        # Additional log area to display more information
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.layout.addWidget(self.log_text)
        
        self.setLayout(self.layout)
        
        self.ros_thread = None
        self.node = None
        self.shutdown_requested = False

    def log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")

    def start_recording(self):
        folder_name = self.folder_line_edit.text().strip()
        if not folder_name:
            self.status_label.setText("Status: Please enter a valid folder name.")
            return
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_path = os.path.join(os.getcwd(), f"{folder_name}_{timestamp}")
        os.makedirs(folder_path, exist_ok=True)
        self.status_label.setText(f"Status: Recording... Saving to folder: {folder_path}")
        self.log(f"Recording started. Folder: {folder_path}")
        
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.shutdown_requested = False
        
        self.ros_thread = threading.Thread(target=self.run_ros, args=(folder_path,), daemon=True)
        self.ros_thread.start()

    def run_ros(self, folder_path):
        rclpy.init(args=None)
        self.node = RecorderNode(folder_path)
        try:
            # Use a polling loop to allow for a controlled shutdown
            while rclpy.ok() and not self.shutdown_requested:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            self.log(f"Error in ROS2 node: {e}")
        finally:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.log("Recording stopped.")
            self.status_label.setText("Status: Idle")
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)

    def stop_recording(self):
        self.log("Stop recording requested.")
        self.shutdown_requested = True

def main(args=None):
    import sys
    app = QApplication(sys.argv)
    gui = RecorderGUI()
    gui.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
