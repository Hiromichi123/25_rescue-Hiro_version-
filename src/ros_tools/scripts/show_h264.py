import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class H264ReceiverNode(Node):
    def __init__(self):
        super().__init__('h264_receiver_node')

        # 设置接收的话题名（根据发送端设置）
        self.subscription = self.create_subscription(
            String,
            '/camera/ground_h264',  # 替换为发送端发布的具体话题名
            self.callback,
            10
        )
        self.subscription  # 防止垃圾回收

        # 初始化视频解码器
        self.decoder = cv2.VideoCapture('appsrc ! h264parse ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        if not self.decoder.isOpened():
            self.get_logger().error('Failed to initialize H.264 decoder.')

    def callback(self, msg):
        try:
            # 将接收到的十六进制字符串还原为二进制数据
            h264_data = bytes.fromhex(msg.data)

            # 将数据写入解码器
            self.decoder.write(h264_data)

            # 从解码器读取帧
            success, frame = self.decoder.read()
            if success:
                # 显示解码后的图像
                cv2.imshow("H264 Stream", frame)
                cv2.waitKey(1)  # 显示窗口刷新

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = H264ReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
