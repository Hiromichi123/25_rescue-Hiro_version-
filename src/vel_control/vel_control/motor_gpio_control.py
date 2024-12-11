import wiringpi as wp
import sys
import time

AIN1 = 3
AIN2 = 4
PWMA = 1
STBY = 5

wp.wiringPiSetup()

wp.pinMode(AIN1, wp.OUTPUT)
wp.pinMode(AIN2, wp.OUTPUT)
wp.pinMode(STBY, wp.OUTPUT)
wp.pinMode(PWMA, wp.PWM_OUTPUT)

wp.pwmSetClock(PWMA, 120)
wp.pwmSetRange(PWMA, 1024)

def motor_forward(speed, target_count):
    wp.digitalWrite(STBY, wp.HIGH)
    wp.digitalWrite(AIN1, wp.LOW)
    wp.digitalWrite(AIN2, wp.HIGH)
    wp.pwmWrite(PWMA, speed)
    
    counter = 0
    while counter < target_count:
        # 可以根据需要加入编码器检测逻辑
        time.sleep(0.01)  # 这里假设一定时间内编码器计数增加
        counter += 1

    wp.digitalWrite(STBY, wp.LOW)

if __name__ == '__main__':
    try:
        speed = int(sys.argv[1])
        target_count = int(sys.argv[2])
        motor_forward(speed, target_count)
    except Exception as e:
        wp.digitalWrite(STBY, wp.LOW)
        sys.exit(1)
