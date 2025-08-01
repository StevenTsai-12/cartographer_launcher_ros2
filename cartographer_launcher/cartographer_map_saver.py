import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import math

class CartographerMapSaver(Node):
    def __init__(self):
        super().__init__('cartographer_map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        # Define parameters
        self.declare_parameter('occupied_thresh', 0.55)
        self.declare_parameter('free_thresh', 0.45)
        self.declare_parameter('filename', 'map')
        # Parse arguments
        self.p_occupied_thresh = self.get_parameter('occupied_thresh').value
        self.p_free_thresh = self.get_parameter('free_thresh').value
        self.p_filename = self.get_parameter('filename').value
        self.saved = False
        self.get_logger().info('Wait the map data...')

    def map_callback(self, msg):
        if self.saved:
            return

        self.get_logger().info(f'Receive the map: {msg.info.width}x{msg.info.height}，resolution: {msg.info.resolution:.3f}')
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # === 符合你 ROS1 的轉換邏輯 ===
        img = np.zeros((height, width), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                value = data[y, x]
                if value >= 0 and value < math.floor(self.p_free_thresh * 100.0):
                    img[y, x] = 254
                elif value >= math.floor(self.p_occupied_thresh * 100.0):
                    img[y, x] = 0
                else:
                    img[y, x] = 205

        # 垂直翻轉 (符合 ROS 標準 PGM 儲存格式)
        img = np.flipud(img)

        # 儲存 PGM
        pgm_file = f'{self.p_filename}.pgm'
        cv2.imwrite(pgm_file, img)
        self.get_logger().info(f'Image saved: {pgm_file}')

        # 儲存 YAML
        yaw = self.quaternion_to_yaw(msg.info.origin.orientation)
        yaml_file = f'{self.p_filename}.yaml'
        yaml_content = {
            'image': f'{self.p_filename}.pgm',
            'mode': 'trinary',
            'resolution': round(float(msg.info.resolution), 4),
            'negate': 0,
            'occupied_thresh': self.p_occupied_thresh,
            'free_thresh': self.p_free_thresh,
        }
        # Origin 單獨輸出
        origin = [
            round(msg.info.origin.position.x, 2),
            round(msg.info.origin.position.y, 2),
            round(yaw, 2)
        ]
        with open(yaml_file, 'w') as f:
            for key, value in yaml_content.items():
                f.write(f"{key}: {value}\n")
                if key == 'resolution':
                    f.write(f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n")
        with open(yaml_file, 'r') as f:
            self.get_logger().info(f'Preview yaml:\n{f.read()}')

        self.get_logger().info(f'Yaml file saved: {yaml_file}')
        self.saved = True
        return

    def quaternion_to_yaw(self, q):
        # 轉換四元數為 Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    saver = CartographerMapSaver()
    rclpy.spin_once(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
