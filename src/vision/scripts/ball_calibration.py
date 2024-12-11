#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from core.msg import Calibrate  # 导入自定义消息
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from enum import Enum
from ros_tools.msg import LidarPose
import math

class Color(Enum):
    RED = 0
    BLUE = 1
    BLACK = 2
    YELLOW = 3

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_calibration')
        self.subscription = self.create_subscription(
            Image,
            '/camera/ground',
            self.image_callback,
            10)
        self.subscription = self.create_subscription(LidarPose, 'lidar_data', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(Calibrate, 'calibrate', 10)
        self.bridge = CvBridge()

        # 定义每种颜色的HSV阀值
        self.color_thresholds = {
            'red': {'H_min1': 0, 'H_max1': 10, 'H_min2': 160, 'H_max2': 180, 'S_min': 70, 'S_max': 255, 'V_min': 50, 'V_max': 255},
            'blue': {'H_min': 100, 'H_max': 140, 'S_min': 50, 'S_max': 255, 'V_min': 50, 'V_max': 255},
            'yellow': {'H_min': 20, 'H_max': 30, 'S_min': 50, 'S_max': 255, 'V_min': 90, 'V_max': 255},
            'black': {'H_min': 0, 'H_max': 180, 'S_min': 30, 'S_max': 60, 'V_min': 0, 'V_max': 30}
        }

        # 定义一个3x3的卷积核
        self.kernel = np.ones((3, 3), np.uint8)

        # 存储LiDAR的位姿信息
        self.lidar_pose = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0
        }

        self.ball_msg = Calibrate(
            delta_x = 0,
            delta_y = 0,
            global_x = 0.0,
            global_y = 0.0,
            color = -1,
            found = False
        )

        # 雷达到摄像头的臂长
        self.length = 0.13

        # 检测历史队列和相关变量
        self.detection_history = deque(maxlen=20)
        self.detected_color_red = deque(maxlen=5)
        self.detected_color_blue = deque(maxlen=5)
        self.detected_color_black = deque(maxlen=5)
        self.detected_color_yellow = deque(maxlen=5)
        self.last_detected_position = None
        self.last_detected_color = None
        self.last_found = False

    def lidar_callback(self, msg):
        self.lidar_pose['x'] = msg.x
        self.lidar_pose['y'] = msg.y
        self.lidar_pose['yaw'] = msg.yaw
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_frame(frame)

    def process_frame(self, frame):
        # 双边滤波
        dst = cv2.bilateralFilter(frame, 5, 50, 50)
        # 将图像转换为HSV颜色空间
        hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

        # 获取摄像头的中心点
        height, width, _ = frame.shape
        camera_center = (width // 2, height // 2)

        # 创建初始的掩膜
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        # 将所有颜色的掩膜合并
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

            # 应用形态学操作
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=5)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=2)

            # 找到所有轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            closest_ball_position = (0, 0)
            min_distance = float('inf')
            ball_found = False
            detected_color_red = False
            detected_color_blue = False
            detected_color_black = False
            detected_color_yellow = False

            """ # 查找最接近中心的小球
            if contours:
                areas = np.array([cv2.contourArea(contour) for contour in contours])
                valid_indices = np.where(areas > 500)[0]

                if len(valid_indices) > 0:
                    valid_contours = [contours[i] for i in valid_indices]
                    centers = [cv2.minEnclosingCircle(contour)[0] for contour in valid_contours]
                    distances = [np.sqrt((center[0] - camera_center[0]) ** 2 + (center[1] - camera_center[1]) ** 2) for center in centers]
                    min_index = np.argmin(distances)
                    closest_ball_position = (int(centers[min_index][0]), int(centers[min_index][1]))
                    min_distance = distances[min_index]
                    ball_found = True """
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 20000:
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    distance = np.sqrt((center[0] - camera_center[0]) ** 2 + (center[1] - camera_center[1]) ** 2)
                    # 找到距离屏幕中心最近的小球
                    if distance < min_distance and radius > 20:
                        min_distance = distance
                        closest_ball_position = center
                        ball_found = True
                        detected_color = Color[color_name.upper()].value if color_name else -1
                        match detected_color:
                            case 0:
                                detected_color_red = True
                                break
                            case 1:
                                detected_color_blue = True
                                break
                            case 2:
                                detected_color_black = True
                                break
                            case 3:
                                detected_color_yellow = True
                                break
                    else:
                        ball_found = False
                        break

            # 更新检测历史
            self.detection_history.append(ball_found)
            self.detected_color_red.append(detected_color_red)
            self.detected_color_blue.append(detected_color_blue)
            self.detected_color_black.append(detected_color_black)
            self.detected_color_yellow.append(detected_color_yellow)

            if ball_found:
                self.get_logger().info(f"Detected ball of color: {color_name}")
            # else:
            #     self.get_logger().info("No ball detected")

            # 判断是否满20帧上有10帧检测到小球
            recent_detections = sum(self.detection_history)
            recent_detections_red = sum(self.detected_color_red)
            recent_detections_blue = sum(self.detected_color_blue)
            recent_detections_black = sum(self.detected_color_black)
            recent_detections_yellow = sum(self.detected_color_yellow)

            if recent_detections >= 10 or recent_detections_red >= 2 or recent_detections_blue >= 2 or recent_detections_black >= 2 or recent_detections_yellow >= 2 and ball_found:
                self.last_detected_position = closest_ball_position
                self.last_detected_color = Color[color_name.upper()].value if color_name else -1
                self.last_found = True
                self.ball_msg = Calibrate(
                    delta_x = camera_center[1] - closest_ball_position[1],
                    delta_y = camera_center[0] - closest_ball_position[0],
                    global_x = (camera_center[1] - closest_ball_position[1])/7200.0 + self.length * math.cos(self.lidar_pose['yaw']),
                    global_y = (camera_center[0] - closest_ball_position[0])/7200.0 + self.length * math.sin(self.lidar_pose['yaw']),
                    color = self.last_detected_color,
                    found = True
                )
                self.publisher_.publish(self.ball_msg)
                self.get_logger().info(f"{self.ball_msg}")
                self.get_logger().info(f"距离:x={(camera_center[1] - closest_ball_position[1])/7200.0 + self.length * math.cos(self.lidar_pose['yaw'])},y={(camera_center[0] - closest_ball_position[0])/7200.0 + self.length * math.sin(self.lidar_pose['yaw'])}")

            else:
                # 不满足条件，返回没有小球
                self.ball_msg = Calibrate(
                    delta_x = 0,
                    delta_y = 0,
                    global_x = 0.0,
                    global_y = 0.0,
                    color = -1,
                    found = False
                )
                
            """ elif recent_detections >= 5 or recent_detections_red >= 2 or recent_detections_blue >= 2 or recent_detections_black >= 2 or recent_detections_yellow >= 2:
                match Color[color_name.upper()].value:
                    case 0:
                        if recent_detections_red >= 2:
                            send = True
                            break
                        else:
                            send = False
                            break
                    case 1:
                        if recent_detections_blue >= 2:
                            send = True
                            break
                        else:
                            send = False
                            break
                    case 2:
                        if recent_detections_black >= 2:
                            send = True
                            break
                        else:
                            send = False
                            break
                    case 3:
                        if recent_detections_blue >= 2:
                            send = True
                            break
                        else:
                            send = False
                            break
                if send:
                    # 没有检测到小球但有30帧满足条件，返回上一帧数据
                    ball_msg = Calibrate(
                        delta_x = (self.last_detected_position[0] - camera_center[0]) if self.last_detected_position else 0,
                        delta_y = (self.last_detected_position[1] - camera_center[1]) if self.last_detected_position else 0,
                        color = self.last_detected_color,
                        found = self.last_found
                    )
                else:
                    continue """
            
def main(args=None):
    rclpy.init(args=args)
    ball_detector = BallDetector()
    rclpy.spin(ball_detector)
    ball_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
