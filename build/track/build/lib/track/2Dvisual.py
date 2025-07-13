import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import threading

class PeoplePositionVisualizer(Node):
    def __init__(self):
        super().__init__('people_position_visualizer')
        
        # 訂閱來自 /people_position 的座標數據
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/people_position',
            self.position_callback,
            10)
        
        self.positions = []  # 存儲歷史座標點
        
        # 啟動繪圖執行緒
        self.thread = threading.Thread(target=self.plot_positions)
        self.thread.daemon = True
        self.thread.start()

    def position_callback(self, msg):
        """處理接收到的 XZ 平面座標並存儲"""
        if len(msg.data) == 2:
            x, z = msg.data
            self.positions.append((x, z))
            self.get_logger().info(f"收到人物位置: X={x:.2f} m, Z={z:.2f} m")

    def plot_positions(self):
        """持續繪製 XZ 平面座標點"""
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            ax.clear()
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Z Position (m)")
            ax.set_title("Real-Time People Position Visualization")
            ax.grid(True)
            
            if self.positions:
                x_vals, z_vals = zip(*self.positions)
                ax.scatter(x_vals, z_vals, c='b', label='Person Position')
                ax.legend()
            
            plt.pause(0.1)
        
        plt.ioff()
        plt.show()


def main():
    rclpy.init()
    node = PeoplePositionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
