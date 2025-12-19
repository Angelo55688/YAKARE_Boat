import pigpio
import time

# 設定PPM輸入引腳
PPM_PIN = 23  # 使用GPIO 23接收PPM信號

# 設置無刷馬達ESC的GPIO引腳
R_PWM_PIN = 12  # 右馬達
L_PWM_PIN = 13  # 左馬達

# PPM信號參數
FRAME_SEPARATION_THRESHOLD = 10000  # 幀分隔閾值(微秒)
MAX_CHANNELS = 6  # FS-iA6B最多支持6個通道

# 搖桿和ESC參數
STICK_MIN = 1015  # 搖桿最小值
ESC_MIN = 1000    # ESC最小值
MAX_PULSE_WIDTH = 2000  # ESC最大脈衝寬度(微秒)
ARM_VALUE = 1000        # ESC啟動值

# 初始化變數
channels = [0] * MAX_CHANNELS  # 存儲通道值
last_tick = 0  # 上次信號變化時間
channel_index = 0  # 當前通道索引
frame_started = False  # 幀開始標誌

# 回調函數處理PPM信號
def ppm_callback(gpio, level, tick):
    global last_tick, channel_index, frame_started, channels
    
    if level == 0:  # 下降沿
        # 計算自上次變化以來的時間(微秒)
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
            # 將脈衝寬度轉換為標準RC值(通常1000-2000)
            channels[channel_index] = pulse_width
            channel_index += 1

# 初始化ESC函數
def init_esc(pi, r_pin, l_pin):
    print("初始化ESC...")
    # 設置引腳為輸出模式
    pi.set_mode(r_pin, pigpio.OUTPUT)
    pi.set_mode(l_pin, pigpio.OUTPUT)
    
    # 發送初始化信號(相當於遙控器油門最低位置)
    pi.set_servo_pulsewidth(r_pin, ARM_VALUE)
    pi.set_servo_pulsewidth(l_pin, ARM_VALUE)
    time.sleep(2)  # 等待ESC初始化完成
    print("ESC初始化完成")

# 控制馬達函數
def control_motors(pi, r_pin, l_pin, throttle_value):
    # 確保值在有效範圍內
    throttle_value = max(ESC_MIN, min(throttle_value, MAX_PULSE_WIDTH))
    
    # 將遙控器值應用到兩個馬達
    pi.set_servo_pulsewidth(r_pin, throttle_value)
    pi.set_servo_pulsewidth(l_pin, throttle_value)

# 主程式
try:
    # 初始化pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("無法連接到pigpio守護進程")
        exit(1)
    
    # 初始化ESC
    init_esc(pi, R_PWM_PIN, L_PWM_PIN)
    
    # 設定PPM輸入引腳和回調
    pi.set_mode(PPM_PIN, pigpio.INPUT)
    callback = pi.callback(PPM_PIN, pigpio.FALLING_EDGE, ppm_callback)
    
    print("開始讀取PPM信號，按Ctrl+C退出")
    print(f"搖桿最小值: {STICK_MIN}, ESC最小值: {ESC_MIN}")
    
    # 主循環顯示通道值並控制馬達
    while True:
        # 顯示所有通道值
        print(f"CH1:{channels[0]:4d} CH2:{channels[1]:4d} CH3:{channels[2]:4d} CH4:{channels[3]:4d} CH5:{channels[4]:4d} CH6:{channels[5]:4d}", end="\r")
        
        # 使用CH3(索引2)的值控制馬達
        throttle = channels[2]
        
        # 只有當搖桿值大於最小值時才控制馬達
        if throttle >= STICK_MIN:
            # 如果需要，可以在這裡添加映射函數，將搖桿值映射到ESC值
            # 例如: 如果搖桿值為1015，ESC值為1000
            control_motors(pi, R_PWM_PIN, L_PWM_PIN, throttle)
        else:
            # 如果搖桿值小於最小值，則將馬達設為最小值
            control_motors(pi, R_PWM_PIN, L_PWM_PIN, ESC_MIN)
        
        time.sleep(0.05)  # 稍微降低循環頻率，減少CPU使用率
        
except KeyboardInterrupt:
    print("\n程式已停止")
    # 停止馬達
    if 'pi' in locals() and pi.connected:
        pi.set_servo_pulsewidth(R_PWM_PIN, ESC_MIN)
        pi.set_servo_pulsewidth(L_PWM_PIN, ESC_MIN)
        time.sleep(0.5)  # 確保停止命令被執行
finally:
    # 清理資源
    if 'callback' in locals():
        callback.cancel()
    if 'pi' in locals() and pi.connected:
        pi.stop()
