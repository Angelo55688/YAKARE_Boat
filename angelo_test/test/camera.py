#export QT_QPA_PLATFORM=xcb

import cv2
import numpy as np
import pigpio
import time

# --- 攝影機與視覺辨識設定 ---
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])
MIN_CONTOUR_AREA = 500

# --- pigpio 與馬達控制設定 ---
PPM_PIN = 23
R_PWM_PIN = 12
L_PWM_PIN = 13
TRIGGER_PIN = 24
FRAME_SEPARATION_THRESHOLD = 10000
MAX_CHANNELS = 6
THROTTLE_MIN_RC = 1015
STEERING_CENTER = 1500
ESC_MIN = 1000
ESC_MAX = 2000
ARM_VALUE = 1000
CH5_MIN_RC = 1150
CH5_MAX_RC = 2000
SERVO_3_CLOCK = 1500
SERVO_1_CLOCK = 800

# --- 新增：PID 控制器設定 ---
# 這是最重要的部分，需要根據實際情況進行調整
KP = 0.01  # 比例增益：主要影響反應速度
KI = 0.01 # 積分增益：消除靜態誤差
KD = 0.2  # 微分增益：抑制震盪，使系統更穩定

# --- 新增：PID 控制器類別 ---
class PIDController:
    """一個簡單的 PID 控制器"""
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0
        self._last_error = 0
        self._last_time = time.time()

    def compute(self, current_value):
        """計算 PID 輸出值"""
        current_time = time.time()
        delta_time = current_time - self._last_time
        
        # 目標是讓 current_value (也就是 x_error) 趨近於 setpoint (0)
        error = self.setpoint - current_value

        # 比例項
        p_term = self.Kp * error

        # 積分項 (加上飽和處理)
        if delta_time > 0:
            self._integral += error * delta_time
        # 限制積分項的大小，防止積分飽和 (Integral Windup)
        if self.output_limits[0] is not None:
            self._integral = max(min(self._integral, self.output_limits[1]), self.output_limits[0])
        i_term = self.Ki * self._integral

        # 微分項
        if delta_time > 0:
            derivative = (error - self._last_error) / delta_time
            d_term = self.Kd * derivative
        else:
            d_term = 0

        # 計算總輸出
        output = p_term + i_term + d_term

        # 限制輸出在安全範圍內
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # 更新狀態
        self._last_error = error
        self._last_time = current_time
        
        return output

    def reset(self):
        """重置控制器狀態"""
        self._integral = 0
        self._last_error = 0
        self._last_time = time.time()

# --- 新增：PID 控制函式 (使用者要求) ---
def pid_control(error, pid_controller):
    """
    使用 PID 控制器計算修正值。
    :param error: 當前的誤差值 (x_error)。
    :param pid_controller: 已初始化的 PIDController 物件。
    :return: 調整後的控制輸出值。
    """
    # 在這個架構中，我們直接將誤差(process variable)傳入compute
    correction = pid_controller.compute(error)
    return correction

# --- 其他既有函式 ---
# (ppm_callback, map_trigger_value, init_all, control_motors 等函式與之前相同)
def ppm_callback(gpio, level, tick):
    global last_tick, channel_index, frame_started, channels
    if level == 0:
        if last_tick == 0: last_tick = tick; return
        pulse_width = pigpio.tickDiff(last_tick, tick); last_tick = tick
        if pulse_width > FRAME_SEPARATION_THRESHOLD: channel_index = 0; frame_started = True; return
        if frame_started and channel_index < MAX_CHANNELS: channels[channel_index] = pulse_width; channel_index += 1
def map_trigger_value(value):
    value = max(CH5_MIN_RC, min(value, CH5_MAX_RC)); return int((CH5_MAX_RC - value) * (SERVO_3_CLOCK - SERVO_1_CLOCK) / (CH5_MAX_RC - CH5_MIN_RC) + SERVO_1_CLOCK)
def init_all(pi, r_pin, l_pin, trigger_pin):
    print("初始化ESC與扣板機馬達..."); pi.set_mode(r_pin, pigpio.OUTPUT); pi.set_mode(l_pin, pigpio.OUTPUT); pi.set_mode(trigger_pin, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(r_pin, ARM_VALUE); pi.set_servo_pulsewidth(l_pin, ARM_VALUE); pi.set_servo_pulsewidth(trigger_pin, SERVO_3_CLOCK); time.sleep(2); print("ESC與扣板機馬達初始化完成")
def control_motors(pi, r_pin, l_pin, throttle_value, steering_value):
    throttle_value = max(ESC_MIN, min(throttle_value, ESC_MAX)); steering_value = max(ESC_MIN, min(steering_value, ESC_MAX))
    steering_diff = steering_value - STEERING_CENTER; left_motor = throttle_value; right_motor = throttle_value
    if steering_diff > 0: right_motor = max(ESC_MIN, min(right_motor - steering_diff, ESC_MAX))
    elif steering_diff < 0: left_motor = max(ESC_MIN, min(left_motor + steering_diff, ESC_MAX))
    pi.set_servo_pulsewidth(l_pin, left_motor); pi.set_servo_pulsewidth(r_pin, right_motor); return left_motor, right_motor

# --- 主程式 ---
try:
    pi = pigpio.pi(); assert pi.connected, "無法連接到 pigpio 守護進程"
    cap = cv2.VideoCapture(0); assert cap.isOpened(), "錯誤：無法開啟攝影機。"
    ret, frame = cap.read(); assert ret, "錯誤：無法讀取第一幀畫面。"
    (height, width) = frame.shape[:2]; frame_center_x = width // 2

    # 初始化 PID 控制器，設定輸出限制避免轉向過度
    pid = PIDController(KP, KI, KD, setpoint=0, output_limits=(-500, 500))
    
    init_all(pi, R_PWM_PIN, L_PWM_PIN, TRIGGER_PIN)
    pi.set_mode(PPM_PIN, pigpio.INPUT)
    callback = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)

    print("="*30 + "\n系統啟動完成，開始 PID 視覺追蹤...\n" + f"畫面中心X: {frame_center_x}, Kp={KP}, Ki={KI}, Kd={KD}\n" + "按 Ctrl+C 退出程式\n" + "="*30)

    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1) + cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        x_error = 0
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > MIN_CONTOUR_AREA:
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"]); cY = int(M["m01"] / M["m00"])
                    x_error = cX - frame_center_x
                    cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
                    cv2.putText(frame, f"Error: {x_error}", (cX-50, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        # 使用 pid_control 函式計算修正值
        correction = pid_control(x_error, pid)

        # 將修正值應用於轉向
        # 注意：我們是將修正值從中心點「減去」，因為PID的error = setpoint - current_value
        auto_steering_value = STEERING_CENTER - int(correction)

        # 其他控制邏輯不變
        throttle_rc = channels[2]; trigger_rc = channels[4]
        if throttle_rc >= THROTTLE_MIN_RC:
            left_motor, right_motor = control_motors(pi, R_PWM_PIN, L_PWM_PIN, throttle_rc, auto_steering_value)
        else:
            left_motor, right_motor = ESC_MIN, ESC_MIN
            pi.set_servo_pulsewidth(R_PWM_PIN, ESC_MIN); pi.set_servo_pulsewidth(L_PWM_PIN, ESC_MIN)
            pid.reset() # 油門歸零時重置PID狀態，避免積分累積

        trigger_pulse = map_trigger_value(trigger_rc) if trigger_rc >= CH5_MIN_RC else SERVO_3_CLOCK
        pi.set_servo_pulsewidth(TRIGGER_PIN, trigger_pulse)
        
        cv2.line(frame, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)
        cv2.imshow('PID Vision Tracking', frame)
        print(f"Throttle:{throttle_rc:4d}|Error:{x_error:4d}|PID_Out:{correction:6.1f}|Steer:{auto_steering_value:4d}|L/R:{left_motor:4d}/{right_motor:4d}\r", end="")

        if cv2.waitKey(1) & 0xFF == ord('q'): break

except (KeyboardInterrupt, AssertionError) as e:
    print(f"\n程式停止: {e}")
finally:
    if 'pi' in locals() and pi.connected:
        pi.set_servo_pulsewidth(R_PWM_PIN, ARM_VALUE); pi.set_servo_pulsewidth(L_PWM_PIN, ARM_VALUE)
        pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_3_CLOCK); time.sleep(0.5)
        if 'callback' in locals(): callback.cancel()
        pi.stop(); print("pigpio 已停止。")
    if 'cap' in locals() and cap.isOpened():
        cap.release(); cv2.destroyAllWindows(); print("攝影機已釋放。")
    print("程式已完全終止。")
