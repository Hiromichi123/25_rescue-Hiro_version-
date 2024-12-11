import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from core.msg import BallDepth
from enum import Enum

class Color(Enum):
    RED = 0
    BLUE = 1
    BLACK = 2
    YELLOW = 3

class BallDepthNode(Node):
    def __init__(self):
        super().__init__('ball_depth_node')
        self.publisher_ = self.create_publisher(BallDepth, 'ball_depth', 10)

        self.depth_subscriber = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.color_subscriber = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        
        self.bridge = CvBridge()
        self.depth_image = None
        self.kernel = np.ones((3, 3), np.uint8)

        self.color_thresholds = {
            'red': {'H_min1': 0, 'H_max1': 10, 'H_min2': 160, 'H_max2': 180, 'S_min': 70, 'S_max': 255, 'V_min': 50, 'V_max': 255},
            'blue': {'H_min': 100, 'H_max': 140, 'S_min': 50, 'S_max': 255, 'V_min': 50, 'V_max': 255},
            'yellow': {'H_min': 20, 'H_max': 30, 'S_min': 50, 'S_max': 255, 'V_min': 90, 'V_max': 255},
            'black': {'H_min': 0, 'H_max': 180, 'S_min': 30, 'S_max': 60, 'V_min': 0, 'V_max': 30}
        }

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = np.where((self.depth_image < 100) | (self.depth_image > 2000), 0,
                                    self.depth_image)  # 只考虖0.1到2米之间的深度

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape

        # 假设的焦距
        f = width / 2

        # 分别处理每种颜色的阈值
        for color_name, thresholds in self.color_thresholds.items():
            if color_name == 'red':
                # 红色阈值分为两个范围
                lower1 = np.array([thresholds['H_min1'], thresholds['S_min'], thresholds['V_min']])
                upper1 = np.array([thresholds['H_max1'], thresholds['S_max'], thresholds['V_max']])
                lower2 = np.array([thresholds['H_min2'], thresholds['S_min'], thresholds['V_min']])
                upper2 = np.array([thresholds['H_max2'], thresholds['S_max'], thresholds['V_max']])
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                # 其他颜色阈值
                lower = np.array([thresholds['H_min'], thresholds['S_min'], thresholds['V_min']])
                upper = np.array([thresholds['H_max'], thresholds['S_max'], thresholds['V_max']])
                mask = cv2.inRange(hsv, lower, upper)

            # 形态学操作去除噪点
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 处理每个轮廓
            for contour in contours:
                ball_info = BallDepth()
                ball_info.found = False
                if cv2.contourArea(contour) > 500:
                    ((u, v), radius) = cv2.minEnclosingCircle(contour)
                    if 50 < radius < 150 and self.depth_image is not None:
                        depth_value = self.depth_image[int(v), int(u)]
                        if depth_value == 0:
                            continue

                        # 计算相机坐标系下的三维坐标 (X, Y, Z)
                        ball_info.depth = int(depth_value)  # 单位：毫米
                        ball_info.x = int((u - width / 2) * depth_value / f)
                        ball_info.y = int((v - height / 2) * depth_value / f)
                        ball_info.found = True
                        ball_info.color = Color[color_name.upper()].value

                        self.publisher_.publish(ball_info)

                    # 在图像上绘制球的中心点
                    cv2.circle(frame, (int(u), int(v)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(u), int(v)), 5, (0, 0, 255), -1)

                    self.publisher_.publish(ball_info)

        # 显示图像
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()