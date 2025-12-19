import pigpio
import time
import sys

# --- GPIO 設定 ---
# PPM 訊號輸入腳位
PPM_PIN = 23
# 無刷馬達 ESC 輸出腳位
R_PWM_PIN = 12  # 右側馬達
L_PWM_PIN = 13  # 左側馬達
# 扣板機伺服馬達腳位
TRIGGER_PIN = 24

# --- ESC 馬達控制參數 (µs) ---
# 這是 ESC 的實際輸出值
MAX_REV = 1000   # 全速後退
NEUTRAL = 1380   # 停止 / 中立點
MAX_FWD = 2000   # 全速前進

# --- 遙控器通道參數 (µs) ---
# 這是從 PPM 訊號讀取到的原始值
THROTTLE_CHANNEL_MIN = 1000  # 油門搖桿最低點
THROTTLE_CHANNEL_NEUTRAL = 1500  # 油門搖桿中立點
THROTTLE_CHANNEL_MAX = 2000  # 油門搖桿最高點
THROTTLE_DEADBAND = 20     # 油門中立區的死區範圍 (+/-)

STEERING_CHANNEL_MIN = 1000    # 方向搖桿最左
STEERING_CHANNEL_CENTER = 1500   # 方向搖桿置中
STEERING_CHANNEL_MAX = 2000    # 方向搖桿最右

# 扣板機伺服馬達映射參數
CH5_MIN = 1150          # CH5 歸零時的值
CH5_MAX = 2000          # CH5 最大值
SERVO_3_CLOCK = 1500    # 3點鐘方向脈寬
SERVO_1_CLOCK = 800     # 1點鐘方向脈寬

# --- PPM 解碼設定 ---
FRAME_SEPARATION_THRESHOLD = 4000  # 幀分隔閾值 (µs)，原 10000 較寬鬆，4000 已足夠
MAX_CHANNELS = 6
channels = [NEUTRAL] * MAX_CHANNELS  # 初始化所有通道值為中立點
last_tick = 0
channel_index = 0

# --- 數值映射函式 ---
def map_range(x, in_min, in_max, out_min, out_max):
    """將一個數值從一個範圍線性映射到另一個範圍"""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- PPM 回調函式 ---
def ppm_callback(gpio, level, tick):
    """處理 PPM 訊號的下降緣來解碼各通道數值"""
    global last_tick, channel_index, channels

    if last_tick == 0:
        last_tick = tick
        return

    pulse_width = pigpio.tickDiff(last_tick, tick)
    last_tick = tick

    if pulse_width > FRAME_SEPARATION_THRESHOLD:
        channel_index = 0
    elif channel_index < MAX_CHANNELS:
        # 進行一些基本的訊號濾波，防止讀到異常值
        if 800 < pulse_width < 2200:
            channels[channel_index] = pulse_width
        channel_index += 1

# --- 初始化函式 ---
def setup_hardware(pi):
    """設定所有 GPIO 腳位模式並初始化 ESC 和伺服馬達"""
    print("▶ 正在初始化硬體...")

    # 設定腳位為輸出模式
    pi.set_mode(R_PWM_PIN, pigpio.OUTPUT)
    pi.set_mode(L_PWM_PIN, pigpio.OUTPUT)
    pi.set_mode(TRIGGER_PIN, pigpio.OUTPUT)

    # *** 重要：有倒退功能的 ESC 應在中立點 (1500µs) 啟動 ***
    print(f"▶ 正在啟動 ESCs (Arming)... 發送 {NEUTRAL}µs 脈衝")
    pi.set_servo_pulsewidth(R_PWM_PIN, NEUTRAL)
    pi.set_servo_pulsewidth(L_PWM_PIN, NEUTRAL)
    
    # 初始化扣板機伺服馬達到 3 點鐘方向
    pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_3_CLOCK)
    
    # 等待 ESC 進入啟動狀態
    time.sleep(2)
    print("✅ ESCs 與伺服馬達初始化完成！")

# --- 清理函式 ---
def cleanup_hardware(pi):
    """程式結束時安全地關閉馬達和 pigpio 連線"""
    print("\n▶ 正在安全關閉...")
    # 將所有馬達和伺服回到中立/初始狀態
    pi.set_servo_pulsewidth(R_PWM_PIN, NEUTRAL)
    pi.set_servo_pulsewidth(L_PWM_PIN, NEUTRAL)
    pi.set_servo_pulsewidth(TRIGGER_PIN, SERVO_3_CLOCK)
    time.sleep(0.5)
    # 完全停止 PWM 訊號
    pi.set_servo_pulsewidth(R_PWM_PIN, 0)
    pi.set_servo_pulsewidth(L_PWM_PIN, 0)
    pi.set_servo_pulsewidth(TRIGGER_PIN, 0)
    print("✅ 硬體已安全關閉。")


# --- 馬達控制邏輯 ---
def control_motors(pi, throttle_ch, steering_ch):
    """根據油門和方向通道的值來控制左右馬達，實現前進、後退和轉向"""
    base_throttle = NEUTRAL
    
    # *** 邏輯修改處：適應反向的油門通道 ***
    # 1. 判斷油門狀態：前進、後退或是在中立死區
    # 現在，搖桿上推 (值變小) 是前進
    if throttle_ch < (THROTTLE_CHANNEL_NEUTRAL - THROTTLE_DEADBAND):
        # 狀態：前進
        # 將遙控器的小值區間 [1470, 1000] 映射到馬達的前進區間 [1500, 2000]
        base_throttle = map_range(throttle_ch, THROTTLE_CHANNEL_NEUTRAL - THROTTLE_DEADBAND, THROTTLE_CHANNEL_MIN, NEUTRAL, MAX_FWD)
    # 搖桿下推 (值變大) 是後退
    elif throttle_ch > (THROTTLE_CHANNEL_NEUTRAL + THROTTLE_DEADBAND):
        # 狀態：後退
        # 將遙控器的大值區間 [1530, 2000] 映射到馬達的後退區間 [1500, 1000]
        base_throttle = map_range(throttle_ch, THROTTLE_CHANNEL_NEUTRAL + THROTTLE_DEADBAND, THROTTLE_CHANNEL_MAX, NEUTRAL, MAX_REV)
    else:
        # 狀態：中立/停止
        base_throttle = NEUTRAL

    # 2. 計算轉向差值
    # 將方向盤的訊號 (-500 to +500) 轉換成一個影響因子
    steering_effect = map_range(steering_ch, STEERING_CHANNEL_MIN, STEERING_CHANNEL_MAX, -300, 300) # 可調整 300 來改變轉向靈敏度

    # 3. 根據前進或後退狀態，應用轉向差值 (差速轉向)
    if base_throttle > NEUTRAL: # 前進時
        # 右轉 (steering_effect > 0) -> 左輪加速，右輪減速
        # 左轉 (steering_effect < 0) -> 左輪減速，右輪加速
        left_motor_pw = base_throttle + steering_effect
        right_motor_pw = base_throttle - steering_effect
    elif base_throttle < NEUTRAL: # 後退時
        # 為了保持直觀的轉向，後退時的邏輯需要反轉
        # 右轉 (steering_effect > 0) -> 左輪減速(更接近NEUTRAL)，右輪加速(更接近MAX_REV)
        # 左轉 (steering_effect < 0) -> 左輪加速，右輪減速
        left_motor_pw = base_throttle + steering_effect
        right_motor_pw = base_throttle - steering_effect
    else: # 靜止時
        # 原地旋轉
        left_motor_pw = NEUTRAL + steering_effect
        right_motor_pw = NEUTRAL - steering_effect

    # 4. 限制最終的脈衝寬度在安全範圍內
    left_motor_pw = int(max(MAX_REV, min(left_motor_pw, MAX_FWD)))
    right_motor_pw = int(max(MAX_REV, min(right_motor_pw, MAX_FWD)))
    
    # 5. 發送訊號給 ESC
    pi.set_servo_pulsewidth(L_PWM_PIN, left_motor_pw)
    pi.set_servo_pulsewidth(R_PWM_PIN, right_motor_pw)

    return left_motor_pw, right_motor_pw

# --- 扣板機伺服馬達控制 ---
def control_trigger(pi, trigger_ch):
    """控制扣板機伺服馬達"""
    # CH5=1150時在3點鐘，CH5=2000時在1點鐘
    if trigger_ch >= CH5_MIN:
        # 將CH5值(1150-2000)線性映射到脈衝寬度(1500-800)
        mapped_pulse = map_range(trigger_ch, CH5_MIN, CH5_MAX, SERVO_3_CLOCK, SERVO_1_CLOCK)
    else:
        mapped_pulse = SERVO_3_CLOCK # 若訊號小於最小值，則回到初始位置

    mapped_pulse = int(max(SERVO_1_CLOCK, min(mapped_pulse, SERVO_3_CLOCK)))
    pi.set_servo_pulsewidth(TRIGGER_PIN, mapped_pulse)
    return mapped_pulse


# --- 主程式 ---
if __name__ == "__main__":
    pi = None
    ppm_cb = None
    try:
        # 連接到 pigpio 守護進程
        pi = pigpio.pi()
        if not pi.connected:
            raise SystemExit("錯誤: 無法連接到 pigpio 守護進程。請確認 'sudo pigpiod' 已執行。")

        # 初始化硬體
        setup_hardware(pi)

        # 設定 PPM 輸入腳位與回調函式
        pi.set_mode(PPM_PIN, pigpio.INPUT)
        ppm_cb = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)

        print("\n✅ 系統準備就緒，開始讀取遙控訊號。按下 Ctrl+C 結束程式。")
        print("-" * 60)

        # 主迴圈
        while True:
            # 從全域變數中獲取最新的通道值
            # CH1: Steering, CH2: Throttle, CH5: Trigger
            steering_channel = channels[0]
            throttle_channel = channels[1] # 油門是通道 2
            trigger_channel = channels[4]

            # 控制馬達 (前進/後退/轉向)
            lm_pw, rm_pw = control_motors(pi, throttle_channel, steering_channel)
            
            # 控制扣板機
            trigger_pw = control_trigger(pi, trigger_channel)

            # 更新顯示的通道資訊
            sys.stdout.write(
                f"\r方向(CH1):{steering_channel:4d} | 油門(CH2):{throttle_channel:4d} | "
                f"左馬達:{lm_pw:4d}µs | 右馬達:{rm_pw:4d}µs | 扣板機:{trigger_pw:4d}µs"
            )





            
            sys.stdout.flush()
            
            time.sleep(0.02) # 迴圈延遲，降低CPU使用率

    except KeyboardInterrupt:
        print("\nℹ️ 收到使用者中斷指令 (Ctrl+C)。")
    except Exception as e:
        print(f"\n❌ 發生未預期錯誤: {e}")
    finally:
        if pi and pi.connected:
            if ppm_cb:
                ppm_cb.cancel() # 移除回調
            cleanup_hardware(pi) # 安全關閉硬體
            pi.stop() # 中斷與 pigpiod 的連線
        print("👋 程式已結束。")
