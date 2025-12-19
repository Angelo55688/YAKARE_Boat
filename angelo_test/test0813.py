import pigpio
import time
import sys

# --- 校準參數 (Calibration) ---
# --- Remote Controller Input Values (µs) ---
RC_THROTTLE_MIN = 1000      # 遙控器油門最低點
RC_THROTTLE_NEUTRAL = 1500  # 遙控器搖桿中立點
RC_THROTTLE_MAX = 2000      # 遙控器油門最高點
RC_THROTTLE_DEADBAND = 100   # 中立死區範圍 (+/- µs)

RC_STEERING_MIN = 1000      # 遙控器方向最左
RC_STEERING_CENTER = 1500   # 遙控器方向中立
RC_STEERING_MAX = 2000      # 遙控器方向最右

# CH5 扳機開關
RC_CH5_MIN = 1150
RC_CH5_MAX = 2000

# --- ESC & Servo Output Values (µs) ---
ESC_MAX_REV = 1000   # 全速倒退
ESC_NEUTRAL = 1518   # 馬達實際停止的中立點
ESC_MAX_FWD = 2000   # 全速前進

# 扣板機伺服馬達
SERVO_POS_REST = 1500  # 初始位置
SERVO_POS_PULL = 800   # 扣板位置

# --- GPIO 腳位 ---
PPM_PIN = 23
R_PWM_PIN = 12
L_PWM_PIN = 13
TRIGGER_PIN = 24

# --- PPM 解碼 ---
FRAME_SEPARATION_THRESHOLD = 4000
MAX_CHANNELS = 6
channels = [RC_THROTTLE_NEUTRAL] * MAX_CHANNELS
last_tick = 0
channel_index = 0

# --- 數值映射 ---
def map_range(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- PPM 回調 ---
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

# --- 硬體初始化 ---
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

# --- 結束清理 ---
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

def control_motors(pi, throttle_ch, steering_ch):
    # 方向死區 (避免搖桿微動導致誤動作)
    STEERING_DEADBAND = 50  

    # === 油門中立判斷 ===
    if abs(throttle_ch - RC_THROTTLE_NEUTRAL) <= RC_THROTTLE_DEADBAND:
        # 油門中立時根據方向桿動作單邊馬達
        if steering_ch > (RC_STEERING_CENTER + STEERING_DEADBAND):
            # 往右 → 左馬達前進
            speed = int(ESC_NEUTRAL + 200)  # 單邊馬達速度，可調
            # left_motor_pw = min(speed, ESC_MAX_FWD)
            right_motor_pw = min(speed, ESC_MAX_FWD)
            left_motor_pw = ESC_NEUTRAL
            # right_motor_pw = ESC_NEUTRAL
        elif steering_ch < (RC_STEERING_CENTER - STEERING_DEADBAND):
            # 往左 → 右馬達前進
            speed = int(ESC_NEUTRAL + 200)
            # right_motor_pw = min(speed, ESC_MAX_FWD)
            left_motor_pw = min(speed, ESC_MAX_FWD)
            right_motor_pw = ESC_NEUTRAL
            # left_motor_pw = ESC_NEUTRAL
        else:
            # 方向桿也在中立 → 停
            left_motor_pw = ESC_NEUTRAL
            right_motor_pw = ESC_NEUTRAL

    else:
        # === 油門有動作 → 正常差速轉向 ===
        if throttle_ch < RC_THROTTLE_NEUTRAL:  # 前進
            base_throttle = map_range(
                throttle_ch,
                RC_THROTTLE_NEUTRAL - RC_THROTTLE_DEADBAND,
                RC_THROTTLE_MIN,
                ESC_NEUTRAL,
                ESC_MAX_FWD
            )
        else:  # 後退
            base_throttle = map_range(
                throttle_ch,
                RC_THROTTLE_NEUTRAL + RC_THROTTLE_DEADBAND,
                RC_THROTTLE_MAX,
                ESC_NEUTRAL,
                ESC_MAX_REV
            )

        # 計算方向影響值
        steering_effect = map_range(
            steering_ch,
            RC_STEERING_MIN, RC_STEERING_MAX,
            -300, 300
        )

        # 差速計算
        left_motor_pw = base_throttle + steering_effect
        right_motor_pw = base_throttle - steering_effect

        # 限制範圍
        left_motor_pw = int(max(ESC_MAX_REV, min(left_motor_pw, ESC_MAX_FWD)))
        right_motor_pw = int(max(ESC_MAX_REV, min(right_motor_pw, ESC_MAX_FWD)))

    # 發送控制到馬達
    pi.set_servo_pulsewidth(L_PWM_PIN, left_motor_pw)
    pi.set_servo_pulsewidth(R_PWM_PIN, right_motor_pw)

    return left_motor_pw, right_motor_pw



# --- 扣板機控制 ---
def control_trigger(pi, trigger_ch):
    if trigger_ch >= RC_CH5_MIN:
        mapped_pulse = map_range(trigger_ch, RC_CH5_MIN, RC_CH5_MAX, SERVO_POS_REST, SERVO_POS_PULL)
    else:
        mapped_pulse = SERVO_POS_REST
    mapped_pulse = int(max(SERVO_POS_PULL, min(mapped_pulse, SERVO_POS_REST)))
    pi.set_servo_pulsewidth(TRIGGER_PIN, mapped_pulse)
    return mapped_pulse

# --- 主程式 ---
if __name__ == "__main__":
    pi = None
    ppm_cb = None
    try:
        pi = pigpio.pi()
        if not pi.connected:
            raise SystemExit("錯誤: 無法連接到 pigpio，請先執行 'sudo pigpiod'")

        setup_hardware(pi)

        pi.set_mode(PPM_PIN, pigpio.INPUT)
        ppm_cb = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)

        print("\n✅ 系統準備就緒。按 Ctrl+C 結束")
        print("-" * 70)

        while True:
            steering_ch = channels[0]  # CH1
            throttle_ch = channels[1]  # CH2
            trigger_ch = channels[4]   # CH5

            lm_pw, rm_pw = control_motors(pi, throttle_ch, steering_ch)
            trig_pw = control_trigger(pi, trigger_ch)

            sys.stdout.write(
                f"\rCH1:{steering_ch:4d} | CH2:{throttle_ch:4d} | "
                f"左馬達:{lm_pw:4d}µs | 右馬達:{rm_pw:4d}µs | 扳機:{trig_pw:4d}µs  "
            )
            sys.stdout.flush()

            time.sleep(0.02)

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
        print("👋 程式結束")
