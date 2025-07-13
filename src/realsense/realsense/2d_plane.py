import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        self.create_subscription(Float32MultiArray, '/world_coordinates', self.callback, 10)

        self.lock = threading.Lock()
        self.latest_points = np.empty((0, 3))
        self.get_logger().info("Matplotlib Visualizer Node Started")

    def callback(self, msg):
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().warn("Data length not divisible by 3!")
            return

        with self.lock:
            self.latest_points = np.array(data, dtype=np.float32).reshape(-1, 3)

    def visualization_loop(self):
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        while True:
            with self.lock:
                points = self.latest_points.copy()

            ax.clear()
            ax.set_title("3D PointCloud (matplotlib)")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(0, 2)

            if points.shape[0] > 0:
                xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
                ax.scatter(xs, ys, zs, c='r', s=5)

            plt.draw()
            plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MatplotlibVisualizer()

    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()

    try:
        node.visualization_loop()  # Keep GUI in main thread
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    ros_spin_thread.join()


if __name__ == '__main__':
    main()