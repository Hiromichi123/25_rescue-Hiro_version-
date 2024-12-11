import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import time

class MotorControllerNode(Node):
    is_task_running = False  # 用于控制是否有任务正在运行

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Int32,
            '/motor',
            self.cmd_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def cmd_callback(self, msg):
        if MotorControllerNode.is_task_running:
            self.get_logger().warn("A motor task is already running, ignoring this command.")
            return

        MotorControllerNode.is_task_running = True  # 标记任务开始
        try:
            if msg.data == 1:
                self.run_motor(511, 70)
            elif msg.data == 2:
                self.run_motor(1023, 165)
                time.sleep(0.5)
                self.run_motor(511, 400)
        finally:
            MotorControllerNode.is_task_running = False  # 标记任务结束

    def run_motor(self, speed, target_count):
        try:
            subprocess.run(['sudo', 'python3', '/home/orangepi/tests/motor_gpio_control.py', str(speed), str(target_count)], check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to run motor: {e}")

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorControllerNode()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
