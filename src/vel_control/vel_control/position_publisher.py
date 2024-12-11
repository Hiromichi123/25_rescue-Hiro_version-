#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist,Vector3, Point  
from std_msgs.msg import Float32MultiArray,Int16MultiArray,String
from ros_tools.msg import LidarPose  # 导入自定义消息类型
import serial  
import struct  
import math  

class SerialNode(Node):  
    def __init__(self, node_name, serial_port_1, serial_port_2, baudrate):  
        super().__init__(node_name)  
         # 初始化两个串口
        self.serial_port_1 = serial_port_1
        self.serial_port_2 = serial_port_2
        self.baudrate = baudrate
        self.ser1 = None
        self.ser2 = None
        self.get_logger().info(f"节点{node_name}已初始化，准备打开串口{serial_port_1}和{serial_port_2}...")

        # 初始化速度属性  
        self.vel_x = 0.0  
        self.vel_y = 0.0  
        self.vel_z = 0.0  

        # 初始化 rc_mode,rc_motor 属性
        self.rc_mode = 0  # 默认为自控模式
        self.rc_motor = 0  # 默认为停止状态

        # 初始化 rc_vel 属性
        self.rc_vel = Vector3()  # 初始化rc_vel为Vector3类型

        # 创建订阅者，订阅 "motor" 话题
        self.motor_subscription = self.create_subscription(
            String,  # 这里使用std_msgs::msg::String类型消息
            "motor",  # 话题名
            self.motor_callback,  # 回调函数
            10  # 队列长度
        )
        self.motor_subscription  # 防止被垃圾回收

        # 创建订阅者，订阅 "rc_control" 话题
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'rc_control',  # 话题名
            self.rc_callback,
            10  # 队列长度
        )
        self.subscription  # 防止警告：未使用变量

        # 创建订阅者  
        self.position_subscription = self.create_subscription(  
            Twist,  
            "position",  
            self.position_callback,  
            10  
        )  
        self.position_subscription  # 防止被垃圾回收  

        # 创建订阅者，订阅"vel"话题，队列长度为10  
        self.velocity_subscription = self.create_subscription(  
            Vector3,  
            "vel",  
            self.velocity_callback,  
            10  
        )  
        self.velocity_subscription  # 防止被垃圾回收

         # 创建订阅者，订阅"LidarPose"话题，获取LidarPose数据
        self.lidar_subscription = self.create_subscription(  
            LidarPose,  
            "lidar_data",  
            self.lidar_callback,  
            10  
        )  
        self.lidar_subscription  # 防止被垃圾回收    
  
        # 打开串口
        try:
            self.ser1 = serial.Serial(self.serial_port_1, self.baudrate, timeout=1)
            self.ser2 = serial.Serial(self.serial_port_2, self.baudrate, timeout=1)
            self.get_logger().info(f"串口{serial_port_1}和{serial_port_2}已打开，波特率为{baudrate}。")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口{serial_port_1}或{serial_port_2}：{e}")
  
        # 定时器，用于周期性地计算和发送速度  
        self.timer = self.create_timer(0.1, self.send_velocity)  # 每0.1秒执行一次，放在最后确保所有初始化完成  
  
        # 存储目标位置和实际位置  
        self.target_position = Twist()  
        self.actual_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # 添加了z轴以匹配send_serial_data的参数  
        self.actual_angle = {'z': 0.0}  # 添加角度信息 

     # 传送带回调函数
    def motor_callback(self, msg):
        command = msg.data.lower().strip()  # 转换为小写并去除两端空格
        if command == "pick":
            self.get_logger().info("收到motor命令：pick")
            # 用户在这里实现pick的具体逻辑
            if self.rc_mode > 0:
                self.get_logger().info("can't pick in rc mode")
            else:
                self.send_motor_data(1)
        elif command == "release":
            self.get_logger().info("收到motor命令：release")
            # 用户在这里实现release的具体逻辑
            if self.rc_button > 0:
                self.get_logger().info("can't release in rc mode")
            else:
                self.send_motor_data(-1)
        else:
            self.get_logger().warning(f"未识别的motor命令：{command}")

    def rc_callback(self, msg):
        # 假设msg.data是一个包含elrs_data的列表
        elrs_data = msg.data  # 这里假设msg.data已经包含了elrs_data的内容

        # 输出接收到的数据长度
        self.get_logger().info(f"Received RC data with {len(elrs_data)} elements")

        # 处理拨杆B (elrs_data[5])
        if (elrs_data[5] <= 720):
            self.get_logger().info("拨杆B: 上 - 发射")
            self.rc_motor = -1
        elif (720 < elrs_data[5] < 1260):
            self.get_logger().info("拨杆B: 中")
            self.rc_motor = 0
        elif (elrs_data[5] >= 1260):
            self.rc_motor = 1
            self.get_logger().info("拨杆B: 下 - 收取")

        # 处理拨杆D (elrs_data[6])
        if (elrs_data[6] <= 992):
            self.rc_mode = 0  # 自控模式
            self.get_logger().info("拨杆D: 上 - 自控")
        else:
            self.rc_mode = 1  # 遥控模式
            self.get_logger().info("拨杆D: 下 - 遥控")

        # 处理右摇杆上下 (elrs_data[1])
        if (elrs_data[1] >= 1650):
            self.rc_vel.x = 0.30
        elif (1420 <= elrs_data[1] < 1650):
            self.rc_vel.x = 0.20
        elif (1190 <= elrs_data[1] < 1420):
            self.rc_vel.x = 0.10
        elif (760 < elrs_data[1] < 1190):
            self.rc_vel.x = 0.0
        elif (530 < elrs_data[1] <= 760):
            self.rc_vel.x = -0.10
        elif (300 < elrs_data[1] <= 530):
            self.rc_vel.x = -0.20
        elif (elrs_data[1] <= 300):
            self.rc_vel.x = -0.30

        # 处理右摇杆左右 (elrs_data[0])
        if (elrs_data[0] <= 300):
            self.rc_vel.y = 0.30
        elif (300 < elrs_data[0] <= 530):
            self.rc_vel.y = 0.20
        elif (530 < elrs_data[0] <= 760):
            self.rc_vel.y = 0.10
        elif (760 < elrs_data[0] < 1190):
            self.rc_vel.y = 0.0
        elif (1190 <= elrs_data[0] < 1420):
            self.rc_vel.y = -0.10
        elif (1420 <= elrs_data[0] < 1650):
            self.rc_vel.y = -0.20
        elif (elrs_data[0] >= 1650):
            self.rc_vel.y = -0.30

        # 处理左摇杆左右 (elrs_data[3]) - 另一个摇杆
        if (elrs_data[3] <= 450):
            self.rc_vel.z = 1.2
            self.get_logger().info("左摇杆左右: 左 - 二挡逆时针转")
        elif (450 < elrs_data[3] <= 750):  
            self.rc_vel.z = 0.8
            self.get_logger().info("左摇杆左右: 左 - 一挡逆时针转")
        elif (750 < elrs_data[3] < 1250):
            self.get_logger().info("左摇杆左右: 中")
            self.rc_vel.z = 0.0
        elif (1250 <= elrs_data[3] < 1550):
            self.rc_vel.z = -0.8
            self.get_logger().info("左摇杆左右: 右 - 一挡顺时针转")
        elif (elrs_data[3] >= 1550):
            self.rc_vel.z = -1.2
            self.get_logger().info("左摇杆左右: 右 - 二挡顺时针转")

    def position_callback(self, msg: Twist):
        self.target_position = msg
        self.get_logger().info(f"接收到目标位置：{self.target_position}")
  
    def lidar_callback(self, msg: LidarPose):  
        # 从LidarPose消息中获取位置和角度  
        self.actual_position['x'] = msg.x  
        self.actual_position['y'] = msg.y  
        self.actual_angle['z'] = msg.yaw  # yaw 作为实际的角度

    def velocity_callback(self, msg: Vector3):  
        self.vel_x = msg.x  
        self.vel_y = msg.y  
        self.vel_z = msg.z
        self.get_logger().info(f"接收到的速度：{self.vel_x},{self.vel_y}, {self.vel_z}")

    def transform_to_local_coordinates(self, x_target, y_target, z_target):
        """ 将全局坐标系中的目标点转换为小车自身坐标系下的目标点 """
        x_car = self.actual_position['x']
        y_car = self.actual_position['y']
        yaw = self.actual_angle['z']

        # 坐标转换公式
        x_target_local = (x_target - x_car) * math.cos(yaw) + (y_target - y_car) * math.sin(yaw)
        y_target_local = -(x_target - x_car) * math.sin(yaw) + (y_target - y_car) * math.cos(yaw)

        # 计算 z_target 和 yaw 的差值
        z_diff = z_target - yaw

        # 归一化角度差，确保角度差在 -π 到 π 之间
        if  z_diff > math.pi:
            z_diff -= 2 * math.pi
        elif z_diff < -math.pi:
            z_diff += 2 * math.pi
        else:
            z_diff = z_diff
        # 计算最终的 z_target_local
        z_target_local = z_diff

        return x_target_local, y_target_local, z_target_local
    
    def send_velocity(self):
        position_button = self.target_position.linear.z
        x_target = self.target_position.linear.x
        y_target = self.target_position.linear.y
        z_target = self.target_position.angular.z
        vx = self.vel_x
        vy = self.vel_y
        vz = self.vel_z
        rc_button = self.rc_mode
        rc_vel_x = self.rc_vel.x
        rc_vel_y = self.rc_vel.y
        rc_vel_z = self.rc_vel.z
        rc_motor = self.rc_motor

        # 将全局坐标系中的目标位置转化为小车自身坐标系下的目标位置
        x_target_local, y_target_local, z_target_local = self.transform_to_local_coordinates(x_target, y_target, z_target)

        if rc_button > 0:
            self.get_logger().info(f"接收到遥控指令，根据遥控指令发布")
            self.send_serial_data(rc_vel_x, rc_vel_y, rc_vel_z)
            self.send_motor_data(rc_motor)
        else:    
            if position_button > 0:
                # 限制最大速度为0.25，并将目标位置限制在一个合理范围内
                x_local = min(max(x_target_local, -0.25), 0.25)
                y_local = min(max(y_target_local, -0.25), 0.25)
                z_target_local = min(max(z_target_local, -1), 1)

                self.send_serial_data(x_local, y_local, z_target_local)
            else:
                self.send_serial_data(vx, vy, vz)

    def send_motor_data(self, motor):
        # 判断 motor 的正负
        if motor > 0:
            # 如果 motor 为正，发送 F 系列指令
            command = f"F{motor}"
        elif motor < 0:
            # 如果 motor 为负，发送 B 系列指令
            command = f"B{-motor}"  # 负数取绝对值
        else:
            self.get_logger().warning("Received motor value is zero, no command sent.")
            return

        # 将指令转为字节数据
        packed_data = command.encode('utf-8')

        # 发送数据到串口 ser2
        try:
            self.ser2.write(packed_data)
            self.get_logger().info(f"已发送电机指令：{command}")
        except serial.SerialException as e:
            self.get_logger().error(f"串口发送电机指令时出错：{e}")

    def send_serial_data(self, x, y, z):  
        def float_to_signed_16bit_packed(value):  
            int_value = round(value * 1000)  
            int_value = max(min(int_value, 32767), -32768)  
            return struct.pack('>h', int_value)  
  
        def calculate_checksum(data):  
            check_sum = 0  
            for byte in data:  
                check_sum ^= byte  
            return check_sum  
  
        packed_data = bytearray([  
            0x7B,  # 帧头  
            0x00,  # 预留位1  
            0x00,  # 预留位2  
        ])  
  
        # 打包X轴速度  
        x_bytes = float_to_signed_16bit_packed(x)  
        packed_data.extend(x_bytes)  
  
        # 打包Y轴速度  
        y_bytes = float_to_signed_16bit_packed(y)  
        packed_data.extend(y_bytes)  
  
        # 打包Z轴速度  
        z_bytes = float_to_signed_16bit_packed(z)  
        packed_data.extend(z_bytes)  
  
        # 添加校验和  
        checksum = calculate_checksum(packed_data) 
        packed_data.append(checksum & 0xFF)  
  
        # 添加帧尾
        packed_data.append(0x7D)  
  
        try:  
            self.ser1.write(packed_data)  
        except serial.SerialException as e:  
            self.get_logger().error(f"串口发送数据时出错：{e}")  
  
    def cleanup(self):  
        if self.ser1 and self.ser1.is_open:  
            self.ser1.close()  
            self.get_logger().info("串口已关闭。")  
        if self.ser2 and self.ser2.is_open:
            self.ser2.close()
            self.get_logger().info("串口已关闭。")

def main(args=None):  
    rclpy.init(args=args)  
    node = SerialNode("serial_node", "/dev/ttyACM0","/dev/ttyS0", 115200)  # 根据实际情况修改串口设备名和波特率  
    rclpy.spin(node)  
    node.cleanup()  
    rclpy.shutdown()  
  
if __name__ == "__main__":
    main()