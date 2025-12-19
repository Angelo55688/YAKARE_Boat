import pigpio
import time

# sudo pigpiod
# cd Desktop/angelo_test/
# sudo python3 example.py

# 设置GPIO引脚
R_PWM_PIN = 12
L_PWM_PIN = 13

# 初始化pigpio
pi = pigpio.pi()

# 设置PWM频率（通常ESC使用50Hz）
PWM_FREQ = 50

# 设置PWM范围（1000-2000是常见的ESC范围）
PWM_RANGE = 2000

# 初始化PWM
pi.set_PWM_frequency(R_PWM_PIN, PWM_FREQ)
pi.set_PWM_frequency(L_PWM_PIN, PWM_FREQ)
pi.set_PWM_range(R_PWM_PIN, PWM_RANGE)
pi.set_PWM_range(L_PWM_PIN, PWM_RANGE)

# 设置初始油门（1000为停止，2000为全速）
def set_throttle(pin, value):
    if value < 1000:
        value = 1000
    elif value > 2000:
        value = 2000
    pi.set_servo_pulsewidth(pin, value)

# 启动ESC（发送最小油门信号）
set_throttle(R_PWM_PIN, 1000)
set_throttle(L_PWM_PIN, 1000)
time.sleep(2)  # 等待ESC初始化

# 逐步增加油门
for throttle in range(1000, 1100, 10):
    set_throttle(R_PWM_PIN, throttle)
    set_throttle(L_PWM_PIN, throttle)
    time.sleep(2)

for throttle in range(1100, 1000, -10):
    set_throttle(R_PWM_PIN, throttle)
    set_throttle(L_PWM_PIN, throttle)
    time.sleep(2)

# set_throttle(R_PWM_PIN, 1100)
# set_throttle(L_PWM_PIN, 1100)
# time.sleep(1)

# 停止电机
set_throttle(R_PWM_PIN, 1000)
set_throttle(L_PWM_PIN, 1000)
time.sleep(1)

# 清理
pi.set_PWM_dutycycle(R_PWM_PIN, 0)
pi.set_PWM_dutycycle(L_PWM_PIN, 0)
pi.stop()