import pigpio
import time

# 設定PPM輸入引腳
PPM_PIN = 23  # 使用GPIO 23接收PPM信號

# 設置無刷馬達ESC的GPIO引腳
R_PWM_PIN = 12  # 右馬達
L_PWM_PIN = 13  # 左馬達

# 扣板機伺服馬達GPIO
TRIGGER_PIN = 24  # 扣板機伺服馬達GPIO

# PPM信號參數
FRAME_SEPARATION_THRESHOLD = 10000  # 幀分隔閾值(微秒)
MAX_CHANNELS = 6  # FS-iA6B最多支持6個通道

# 搖桿和ESC參數
THROTTLE_MIN = 1015  # CH3油門搖桿最小值
STEERING_CENTER = 1500  # CH1方向搖桿中間值
ESC_MIN = 1000    # ESC最小值
MAX_PULSE_WIDTH = 2000  # ESC最大脈衝寬度(微秒)
ARM_VALUE = 1000        # ESC啟動值

# 伺服馬達參數
SERVO_MIN = 1000
SERVO_MAX = 2000
SERVO_CENTER = 1500

# 扣板機伺服馬達映射參數
CH5_MIN = 1150          # CH5 歸零時的值
CH5_MAX = 2000          # CH5 最大值
SERVO_3_CLOCK = 1500    # 3點鐘方向脈寬
SERVO_1_CLOCK = 800   # 1點鐘方向脈寬

# 初始化變數
channels = [0] * MAX_CHANNELS  # 存儲通道值
last_tick = 0  # 上次信號變化時間
channel_index = 0  # 當前通道索引
frame_started = False  # 幀開始標誌

# 回調函數處理PPM信號
def ppm_callback(gpio, level, tick):
    global last_tick, channel_index, frame_started, channels

    if level == 0:  # 下降沿
        if last_tick == 0:
            last_tick = tick
            return

        pulse_width = pigpio.tickDiff(last_tick, tick)
        last_tick = tick

        # 檢測幀分隔
        if pulse_width > FRAME_SEPARATION_THRESHOLD:
            channel_index = 0
            frame_started = True
            return

        # 存儲通道值
        if frame_started and channel_index < MAX_CHANNELS:
            channels[channel_index] = pulse_width
            channel_index += 1

# 扣板機伺服馬達脈寬映射函數（數值增大往1點鐘方向）
def map_trigger_value(value):
    # 將CH5值(1150-2000)線性映射到脈衝寬度(1500-1000)
    value = max(CH5_MIN, min(value, CH5_MAX))  # 限制範圍
    return int((CH5_MAX - value) * (SERVO_3_CLOCK - SERVO_1_CLOCK) / (CH5_MAX - CH5_MIN) + SERVO_1_CLOCK)

# 初始化ESC與伺服馬達
def init_esc(pi, r_pin, l_pin, trigger_pin):
    print("初始化ESC與扣板機馬達...")
    pi.set_mode(r_pin, pigpio.OUTPUT)
    pi.set_mode(l_pin, pigpio.OUTPUT)
    pi.set_mode(trigger_pin, pigpio.OUTPUT)

    # 發送初始化信號
    pi.set_servo_pulsewidth(r_pin, ARM_VALUE)
    pi.set_servo_pulsewidth(l_pin, ARM_VALUE)
    pi.set_servo_pulsewidth(trigger_pin, SERVO_3_CLOCK)  # 初始3點鐘方向
    time.sleep(2)
    print("ESC與扣板機馬達初始化完成")

# 控制馬達函數
def control_motors(pi, r_pin, l_pin, throttle_value, steering_value):
    throttle_value = max(ESC_MIN, min(throttle_value, MAX_PULSE_WIDTH))
    steering_diff = steering_value - STEERING_CENTER
    left_motor = throttle_value
    right_motor = throttle_value

    if steering_diff > 0:  # 右轉
        left_motor = max(ESC_MIN, min(left_motor + steering_diff, MAX_PULSE_WIDTH))
    elif steering_diff < 0:  # 左轉
        right_motor = max(ESC_MIN, min(right_motor - steering_diff, MAX_PULSE_WIDTH))

    pi.set_servo_pulsewidth(l_pin, left_motor)
    pi.set_servo_pulsewidth(r_pin, right_motor)
    return left_motor, right_motor

# 主程式
try:
    pi = pigpio.pi()
    if not pi.connected:
        print("無法連接到pigpio守護進程")
        exit(1)

    # 初始化
    init_esc(pi, R_PWM_PIN, L_PWM_PIN, TRIGGER_PIN)

    # 設定PPM輸入引腳和回調
    pi.set_mode(PPM_PIN, pigpio.INPUT)
    callback = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)

    print("開始讀取PPM信號，按Ctrl+C退出")
    print(f"油門最小值: {THROTTLE_MIN}, 方向中心值: {STEERING_CENTER}, ESC最小值: {ESC_MIN}")

    while True:
        throttle = channels[2]  # CH3
        steering = channels[0]  # CH1
        trigger_value = channels[4]  # CH5

        print(f"CH1:{steering:4d} CH2:{channels[1]:4d} CH3:{throttle:4d} CH4:{channels[3]:4d} CH5:{trigger_value:4d} CH6:{channels[5]:4d}", end=" | ")

        # 控制左右馬達
        if throttle >= THROTTLE_MIN:
            left_motor, right_motor = control_motors(pi, R_PWM_PIN, L_PWM_PIN, throttle, steering)
            print(f"左馬達:{left_motor:4d} 右馬達:{right_motor:4d}", end=" | ")
        else:
            pi.set_servo_pulsewidth(R_PWM_PIN, ESC_MIN)
            pi.set_servo_pulsewidth(L_PWM_PIN, ESC_MIN)
            print(f"左馬達:{ESC_MIN:4d} 右馬達:{ESC_MIN:4d}", end=" | ")

        # 控制扣板機伺服馬達（MG996R）
        # CH5=1150時在3點鐘，CH5=2000時在1點鐘
        if trigger_value >= CH5_MIN:
            mapped_pulse = map_trigger_value(trigger_value)
            pi.set_servo_pulsewidth(TRIGGER_PIN, mapped_pulse)
            print(f"扣板機:{mapped_pulse:4d}μs", end="")
        else:
            pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_3_CLOCK)
            print(f"扣板機:{SERVO_3_CLOCK:4d}μs", end="")

        print("\r", end="")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n程式已停止")
    if 'pi' in locals() and pi.connected:
        pi.set_servo_pulsewidth(R_PWM_PIN, ESC_MIN)
        pi.set_servo_pulsewidth(L_PWM_PIN, ESC_MIN)
        pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_3_CLOCK)
        time.sleep(0.5)
finally:
    if 'callback' in locals():
        callback.cancel()
    if 'pi' in locals() and pi.connected:
        pi.stop()
