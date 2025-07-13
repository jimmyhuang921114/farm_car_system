from rclpy.node import Node
import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

class BoxEdgeDetect(Node):
    def __init__(self):
        super().__init__('box_edge_detect')
        self.bridge = CvBridge()
        self.color_img = self.create_subscription(Image, '/farm/color',self.color_callback, 10)
        self.depth_img = self.create_subscription(Image, '/farm/depth',self.depth_callback, 10)
        # self.text_coordinate = self.create_subscription(Pose, 'word/text_coordinate',self.text_coordinate_callback, 10)
        self.box_edge = self.create_publisher(Pose, 'visual/box_edge', 10)
        self.color_img_data = None
        self.depth_img_data = None
        self.box_edge_data = Pose()

    def color_callback(self, msg):
        self.color_img_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.get_logger().info('Color image received')
        if  self.color_img_data is None:
            self.get_logger().info('Color image is None')
        self.box_edge_detect()
    
    def depth_callback(self, msg):
        self.depth_img_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # self.get_logger().info('Depth image received')
        if  self.depth_img_data is None:
            self.get_logger().info('Depth image is None')

    # def text_coordinate_callback(self, msg):#text coordinate
    #     self.text_coordinate_data = msg
    #     if self.text_coordinate_data is None:
    #         self.get_logger().info('Text coordinate is None')
    #     self.position_coordinate = self.text_coordinate_data.position
    #     self.orientation_coordinate = self.text_coordinate_data.orientation

    def ShapeDetect(self,edges_img):
        contours,hierarchy = cv2.findContours(edges_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for obj in contours:
            # area = cv2.contourArea(obj) #calculate the area
            
            cv2.drawContours(edges_img, obj, -1, (0, 255, 0), 3) #draw the contours
            perimeter  = cv2.arcLength(obj, True) #calculate the perimeter
            approx = cv2.approxPolyDP(obj, 0.02 * perimeter, True) #approximate the contour
            # corner_number  = len(approx) #calculate the number of corners
            x, y, w, h = cv2.boundingRect(obj) #calculate the bounding box
            if w > 200 and h > 200: #if the number of corners is 4 and the width and height are greater than 400
                # self.get_logger().warn('Detect shaoe have no four corners')
                self.get_logger().info('Detect shape is a rectangle')
                self.get_logger().info('x: {}, y: {}'.format(x, y))
                cv2.rectangle(edges_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cv2.drawContours(edges_img, approx, -1, (0, 0, 255), 3)
            else:
                continue
            # cv2.putText(edges_img, 'x: {}, y: {}'.format(x, y), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow('Edges', edges_img)
            cv2.waitKey(1)
    
    def box_edge_detect(self):
        if self.color_img_data is None or self.depth_img_data is None:
            self.get_logger().warn('Color image or depth image is None')
            return
        # Convert the color image to grayscale
        gray = cv2.cvtColor(self.color_img_data, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to the grayscale image
        blurred = cv2.GaussianBlur(gray, (7, 7), 0) #gaussian filiter
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(blurred)
        # Perform Canny edge detection
        edges = cv2.Canny(enhanced, 50, 150)
        #use keep ratio to resize the image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        fixed = cv2.dilate(edges, kernel, iterations=4)
        closed = cv2.morphologyEx(fixed, cv2.MORPH_CLOSE, kernel)
        cleand = cv2.erode(closed, kernel,iterations=2)
        # Find contours in the edge-detected image
        self.ShapeDetect(cleand)


def main(args=None):
    rclpy.init(args=args)
    box_edge_detect = BoxEdgeDetect()
    rclpy.spin(box_edge_detect)
    box_edge_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()