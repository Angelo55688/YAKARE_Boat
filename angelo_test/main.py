#export QT_QPA_PLATFORM=xcb

from flask import Flask, Response, render_template_string
import cv2
import threading
import time
import numpy as np
import pigpio
import sys

# ================== 控制參數 ==================
ESC_MAX_REV = 1000
ESC_NEUTRAL = 1518
ESC_MAX_FWD = 2000

SERVO_POS_REST = 1500
SERVO_POS_PULL = 800

PPM_PIN = 23
R_PWM_PIN = 12
L_PWM_PIN = 13
TRIGGER_PIN = 24

FRAME_SEPARATION_THRESHOLD = 10000
MAX_CHANNELS = 6
THROTTLE_MIN_RC = 1015
CH5_MIN_RC = 1150
CH5_MAX_RC = 2000

# PID 參數
KP = 0.09  # 比例增益：主要影響反應速度
KI = 0.01 # 積分增益：消除靜態誤差
KD = 0.2  # 微分增益：抑制震盪，使系統更穩定

# 視覺辨識設定
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])
MIN_CONTOUR_AREA = 500

# RC 校準
RC_THROTTLE_MIN = 1000
RC_THROTTLE_NEUTRAL = 1500
RC_THROTTLE_MAX = 2000
RC_THROTTLE_DEADBAND = 100

RC_STEERING_MIN = 1000
RC_STEERING_CENTER = 1500
RC_STEERING_MAX = 2000

RC_CH5_MIN = 1150
RC_CH5_MAX = 2000

# ================== 基本函式 ==================
def map_range(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setup_hardware(pi):
    print("▶ 初始化硬體...")
    pi.set_mode(R_PWM_PIN, pigpio.OUTPUT)
    pi.set_mode(L_PWM_PIN, pigpio.OUTPUT)
    pi.set_mode(TRIGGER_PIN, pigpio.OUTPUT)
    print(f"▶ 啟動 ESCs，發送 {ESC_NEUTRAL}µs 中立訊號")
    pi.set_servo_pulsewidth(R_PWM_PIN, ESC_NEUTRAL)
    pi.set_servo_pulsewidth(L_PWM_PIN, ESC_NEUTRAL)
    pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_POS_REST)
    time.sleep(2)
    print("✅ ESC 和伺服馬達初始化完成。")

def cleanup_hardware(pi):
    print("\n▶ 安全關閉...")
    pi.set_servo_pulsewidth(R_PWM_PIN, ESC_NEUTRAL)
    pi.set_servo_pulsewidth(L_PWM_PIN, ESC_NEUTRAL)
    pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_POS_REST)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(R_PWM_PIN, 0)
    pi.set_servo_pulsewidth(L_PWM_PIN, 0)
    pi.set_servo_pulsewidth(TRIGGER_PIN, 0)
    print("✅ 硬體已安全關閉。")

# ================== PID 控制器 ==================
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-500, 500)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0
        self._last_error = 0
        self._last_time = time.time()

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self._last_time
        error = self.setpoint - current_value
        p_term = self.Kp * error
        if delta_time > 0:
            self._integral += error * delta_time
        self._integral = max(self.output_limits[0], min(self._integral, self.output_limits[1]))
        i_term = self.Ki * self._integral
        d_term = self.Kd * ((error - self._last_error)/delta_time) if delta_time > 0 else 0
        output = p_term + i_term + d_term
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        self._last_error = error
        self._last_time = current_time
        return output

    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_time = time.time()

def pid_control(error, pid_controller):
    return pid_controller.compute(error)

# ================== 視覺檢測 ==================
def get_red_error(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1) + cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    x_error = None
    cX = cY = None
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_contour) > MIN_CONTOUR_AREA:
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                frame_center_x = frame.shape[1] // 2
                x_error = cX - frame_center_x
    return x_error, cX, cY

# ================== pigpio + 馬達/trigger/PPM控制 ==================
channels = [RC_THROTTLE_NEUTRAL] * MAX_CHANNELS
last_tick = 0
channel_index = 0

def ppm_callback(gpio, level, tick):
    global last_tick, channel_index, channels
    if last_tick == 0:
        last_tick = tick
        return
    pulse_width = pigpio.tickDiff(last_tick, tick)
    last_tick = tick
    if pulse_width > FRAME_SEPARATION_THRESHOLD:
        channel_index = 0
    elif channel_index < MAX_CHANNELS:
        if 800 < pulse_width < 2200:
            channels[channel_index] = pulse_width
        channel_index += 1

def control_motors(pi, throttle_ch, steering_ch):
    STEERING_DEADBAND = 50
    if abs(throttle_ch - RC_THROTTLE_NEUTRAL) <= RC_THROTTLE_DEADBAND:
        if steering_ch > (RC_STEERING_CENTER + STEERING_DEADBAND):
            speed = int(ESC_NEUTRAL + 200)
            right_motor_pw = min(speed, ESC_MAX_FWD)
            left_motor_pw = ESC_NEUTRAL
        elif steering_ch < (RC_STEERING_CENTER - STEERING_DEADBAND):
            speed = int(ESC_NEUTRAL + 200)
            left_motor_pw = min(speed, ESC_MAX_FWD)
            right_motor_pw = ESC_NEUTRAL
        else:
            left_motor_pw = ESC_NEUTRAL
            right_motor_pw = ESC_NEUTRAL
    else:
        if throttle_ch < RC_THROTTLE_NEUTRAL:
            base_throttle = map_range(
                throttle_ch, RC_THROTTLE_NEUTRAL - RC_THROTTLE_DEADBAND, RC_THROTTLE_MIN,
                ESC_NEUTRAL, ESC_MAX_FWD)
        else:
            base_throttle = map_range(
                throttle_ch, RC_THROTTLE_NEUTRAL + RC_THROTTLE_DEADBAND, RC_THROTTLE_MAX,
                ESC_NEUTRAL, ESC_MAX_REV)
        steering_effect = map_range(
            steering_ch, RC_STEERING_MIN, RC_STEERING_MAX, -300, 300 )
        left_motor_pw = base_throttle + steering_effect
        right_motor_pw = base_throttle - steering_effect
        left_motor_pw = int(max(ESC_MAX_REV, min(left_motor_pw, ESC_MAX_FWD)))
        right_motor_pw = int(max(ESC_MAX_REV, min(right_motor_pw, ESC_MAX_FWD)))
    pi.set_servo_pulsewidth(L_PWM_PIN, left_motor_pw)
    pi.set_servo_pulsewidth(R_PWM_PIN, right_motor_pw)
    return left_motor_pw, right_motor_pw

def control_trigger(pi, trigger_ch):
    if trigger_ch >= RC_CH5_MIN:
        mapped_pulse = map_range(trigger_ch, RC_CH5_MIN, RC_CH5_MAX, SERVO_POS_REST, SERVO_POS_PULL)
    else:
        mapped_pulse = SERVO_POS_REST
    mapped_pulse = int(max(SERVO_POS_PULL, min(mapped_pulse, SERVO_POS_REST)))
    pi.set_servo_pulsewidth(TRIGGER_PIN, mapped_pulse)
    return mapped_pulse

# ================== Flask 串流 ==================
lock = threading.Lock()
app = Flask(__name__)
video_capture = None
output_frame = None

def capture_frames(pi, pid_controller):
    global video_capture, output_frame, lock, channels
    while True:
        if video_capture is None or not video_capture.isOpened():
            time.sleep(0.5)
            continue
        ret, frame = video_capture.read()
        if ret:
            x_error, cX, cY = get_red_error(frame)
            if x_error is None: x_error = 0
            correction = pid_control(x_error, pid_controller)
            auto_steering_value = ESC_NEUTRAL - int(correction)

            throttle_ch = channels[1]
            trigger_ch = channels[4]

            lm, rm = control_motors(pi, throttle_ch, auto_steering_value)
            trig_pw = control_trigger(pi, trigger_ch)

            height, width = frame.shape[:2]
            if cX and cY:
                cv2.circle(frame, (cX, cY), 7, (0,255,0), -1)
                cv2.putText(frame, f"Error: {x_error}", (cX-50, cY-25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            status_txt = f"T:{throttle_ch}|Err:{x_error}|PID:{correction:.1f}|Steer:{auto_steering_value}|L/R:{lm}/{rm}|Trig:{trig_pw}"
            cv2.putText(frame, status_txt, (12,24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, ts, (10, height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            with lock:
                output_frame = frame.copy()
        else:
            time.sleep(0.5)

def generate_frame():
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n'
        time.sleep(0.03)

@app.route("/")
def index():
    html_template = """
    <html><head><title>即時視覺追蹤</title></head>
    <body style="background:#222;color:#fff;">
        <h1>PID 視覺追蹤串流</h1>
        <img src="{{ url_for('video_feed') }}" style="width:90%;">
    </body></html>
    """
    return render_template_string(html_template)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frame(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# ================== 合併主程式 ==================
if __name__ == "__main__":
    pi = None
    ppm_cb = None
    video_capture = None
    pid = None
    try:
        pi = pigpio.pi()
        if not pi.connected:
            raise SystemExit("錯誤: 無法連接到 pigpio，請先執行 'sudo pigpiod'")

        setup_hardware(pi)

        pi.set_mode(PPM_PIN, pigpio.INPUT)
        ppm_cb = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)

        pid = PIDController(KP, KI, KD, setpoint=0, output_limits=(-500, 500))
        video_capture = cv2.VideoCapture(0)
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not video_capture.isOpened():
            raise RuntimeError("無法開啟攝影機")

        t = threading.Thread(target=capture_frames, args=(pi, pid))
        t.daemon = True
        t.start()

        print("\n✅ 系統準備就緒。Ctrl+C 結束")
        print("請用瀏覽器連線 http://<樹莓派IP>:8080")
        print("-" * 70)

        # 給 Flask 啟動主程式，不會跳出 while
        app.run(host='0.0.0.0', port=8080, threaded=True)

    except KeyboardInterrupt:
        print("\nℹ️ 使用者中斷")
    except Exception as e:
        print(f"\n❌ 發生錯誤: {e}")
    finally:
        if pi and pi.connected:
            if ppm_cb:
                ppm_cb.cancel()
            cleanup_hardware(pi)
            pi.stop()
        if video_capture and video_capture.isOpened():
            video_capture.release()
        print("👋 程式結束")
