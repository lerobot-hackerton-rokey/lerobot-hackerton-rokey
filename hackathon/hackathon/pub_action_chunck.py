import rclpy
from rclpy.node import Node
from inference_interfaces.msg import ActionChunk
import time

class ChunkPublisher(Node):
    def __init__(self):
        super().__init__('chunk_publisher')
        self.publisher_ = self.create_publisher(ActionChunk, '/joint_command_chunk', 10)
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        msg = ActionChunk()
        msg.rows = 10
        msg.cols = 6
        # msg.data = [0.1] * (msg.rows * msg.cols)
        # print(msg.data)

        
        data_list = []
        for i in range(msg.rows):
            for j in range(msg.cols):
                data_list.append((i+1)/100 * (-1))
                # data_list.append((i+1)/100 * (1))
        
        print(data_list)    
        print("===")
        msg.data = data_list
        print(msg.data)
        
        self.publisher_.publish(msg)
        self.get_logger().info('📤 Sent ActionChunk once.')

        # 노드와 타이머 정리
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ChunkPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
