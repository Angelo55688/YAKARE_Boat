import serial
import struct
import time
import crcmod.predefined
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv
import threading
import sys
import pigpio

# --- 卡爾曼濾波器類別 (KalmanFilter Class) ---
class KalmanFilter:
    """一個簡單的一維卡爾曼濾波器"""
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.error_estimate = 1.0
        
    def update(self, measurement):
        prediction = self.estimate
        prediction_error = self.error_estimate + self.process_variance
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.error_estimate = (1 - kalman_gain) * prediction_error
        return self.estimate

# --- IMU 處理器類別 (IMU Processor Class) ---
class IMUProcessor:
    """在背景執行緒中讀取和處理 IMU 數據"""
    def __init__(self, port, baudrate):
        self.serial_port = port
        self.serial_bps = baudrate
        self.ser = None
        self.df_head = b'\xfc'
        self.addr_imu = 0x40
        self.imu_len = 0x38
        self.mag_scale = [1.0037, 0.9600, 1.0394]
        self.mag_offset = [-28.875, 273.75, 306.0]
        self.initial_heading_offset = None
        self.kf_heading = KalmanFilter(process_variance=0.01, measurement_variance=5.0)
        self.heading_buffer = []
        self.buffer_size = 5
        self.crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
        
        # 新增：陀螺儀偏差變數
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        # 擴展最新的數據存儲
        self.latest_heading_filtered = 0.0
        self.latest_heading_raw = 0.0
        self.latest_gyro_x = 0.0
        self.latest_gyro_y = 0.0
        self.latest_gyro_z = 0.0
        self.latest_mag_x_calib = 0.0
        self.latest_mag_y_calib = 0.0
        
        self.is_running = False
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_bps, timeout=1)
            print(f"✅ 成功連接到 IMU 於 {self.serial_port}")
            return True
        except serial.SerialException as e:
            print(f"❌ 錯誤: 無法開啟 IMU 序列埠 {self.serial_port}. {e}")
            return False

    def calibrate_gyro(self, num_samples=200):
        """
        新增：陀螺儀校準函數。
        在開始時測量靜態偏差。
        """
        print("▶️ 開始陀螺儀校準，請保持船隻靜止...")
        gyro_data_x, gyro_data_y, gyro_data_z = [], [], []
        
        samples_collected = 0
        while samples_collected < num_samples:
            try:
                if self.ser.read(1) == self.df_head:
                    header_data = self.ser.read(6)
                    if len(header_data) < 6: continue
                    addr, length, _, head_crc8, _, _ = struct.unpack('<BBBBBB', header_data)
                    header_to_check = self.df_head + header_data[:3]
                    if self.crc8_func(header_to_check) == head_crc8 and addr == self.addr_imu and length == self.imu_len:
                        payload = self.ser.read(length)
                        self.ser.read(1) # 清除結尾
                        if len(payload) == length:
                            gx, gy, gz = struct.unpack_from('<fff', payload, 12)
                            gyro_data_x.append(math.degrees(gx))
                            gyro_data_y.append(math.degrees(gy))
                            gyro_data_z.append(math.degrees(gz))
                            samples_collected += 1
            except Exception:
                continue # 忽略校準期間的錯誤

        if not gyro_data_z: # 如果沒有採集到數據
             print("❌ 陀螺儀校準失敗，將使用 0 作為偏差。")
             return

        self.gyro_bias_x = np.mean(gyro_data_x)
        self.gyro_bias_y = np.mean(gyro_data_y)
        self.gyro_bias_z = np.mean(gyro_data_z)
        
        print("✅ 陀螺儀校準完成！")
        print(f"  - 偏差 (X, Y, Z): ({self.gyro_bias_x:.2f}, {self.gyro_bias_y:.2f}, {self.gyro_bias_z:.2f}) °/s")

    def start_reading_thread(self):
        self.is_running = True
        thread = threading.Thread(target=self._read_loop)
        thread.daemon = True
        thread.start()
        print("▶ IMU 讀取執行緒已啟動")

    def stop_reading(self):
        self.is_running = False
        if self.ser:
            self.ser.close()
        print("⏹️ IMU 讀取已停止")

    def get_latest_data(self):
        with self.lock:
            return (self.latest_heading_filtered, self.latest_heading_raw,
                    self.latest_gyro_x, self.latest_gyro_y, self.latest_gyro_z,
                    self.latest_mag_x_calib, self.latest_mag_y_calib)

    def _read_loop(self):
        while self.is_running:
            try:
                if self.ser.read(1) != self.df_head: continue
                header_data = self.ser.read(6)
                if len(header_data) < 6: continue
                addr, length, _, head_crc8, _, _ = struct.unpack('<BBBBBB', header_data)
                header_to_check = self.df_head + header_data[:3]
                if self.crc8_func(header_to_check) != head_crc8: continue
                if addr == self.addr_imu and length == self.imu_len:
                    payload = self.ser.read(length)
                    self.ser.read(1)
                    if len(payload) == length:
                        self._parse_imu_payload(payload)
            except (serial.SerialException, OSError):
                print("IMU 序列埠錯誤，2秒後嘗試重新連接...")
                time.sleep(2)
                self.connect()
            except Exception as e:
                print(f"IMU 讀取時發生未知錯誤: {e}")
                time.sleep(1)

    def _parse_imu_payload(self, data):
        gyro_x_raw, gyro_y_raw, gyro_z_raw = struct.unpack_from('<fff', data, 12)
        mag_x, mag_y, _ = struct.unpack_from('<fff', data, 24)
        
        # 應用陀螺儀校準
        calibrated_gyro_x = math.degrees(gyro_x_raw) - self.gyro_bias_x
        calibrated_gyro_y = math.degrees(gyro_y_raw) - self.gyro_bias_y
        calibrated_gyro_z = math.degrees(gyro_z_raw) - self.gyro_bias_z

        calib_x = (mag_x - self.mag_offset[0]) * self.mag_scale[0]
        calib_y = (mag_y - self.mag_offset[1]) * self.mag_scale[1]
        
        heading_rad = math.atan2(-calib_y, calib_x)
        heading_deg = (math.degrees(heading_rad) + 360) % 360
        if self.initial_heading_offset is None:
            self.initial_heading_offset = heading_deg
        heading_raw = (heading_deg - self.initial_heading_offset + 360) % 360
        
        self.heading_buffer.append(heading_raw)
        if len(self.heading_buffer) > self.buffer_size:
            self.heading_buffer.pop(0)
        
        sorted_buffer = np.sort(self.heading_buffer)
        if sorted_buffer[-1] - sorted_buffer[0] > 180:
            temp_buffer = np.array([h + 360 if h < 180 else h for h in self.heading_buffer])
            heading_median = np.median(temp_buffer) % 360
        else:
            heading_median = np.median(self.heading_buffer)
        
        heading_filtered = self.kf_heading.update(heading_median) % 360
        
        with self.lock:
            self.latest_heading_filtered = heading_filtered
            self.latest_heading_raw = heading_raw
            self.latest_gyro_x = calibrated_gyro_x
            self.latest_gyro_y = calibrated_gyro_y
            self.latest_gyro_z = calibrated_gyro_z
            self.latest_mag_x_calib = calib_x
            self.latest_mag_y_calib = calib_y

# --- 馬達與遙控器處理類別 (Motor & RC Controller Class) ---
class MotorController:
    # ... (此類別的程式碼保持不變)
    """在背景執行緒中處理PPM輸入並控制馬達，同時估算速度"""
    
    # --- 校準與設定 (請根據您的硬體修改) ---
    # *** 速度校準參數 ***
    # TODO: 透過實驗測量您的船在油門全開時的最大速度 (公尺/秒)
    MAX_FORWARD_SPEED_MPS = 2.0  # 估計的最大前進速度 (m/s)
    MAX_REVERSE_SPEED_MPS = 0.5  # 估計的最大後退速度 (m/s)

    # 遙控器輸入 (µs)
    RC_THROTTLE_MIN = 1000
    RC_THROTTLE_NEUTRAL = 1500
    RC_THROTTLE_MAX = 2000
    RC_THROTTLE_DEADBAND = 100
    RC_STEERING_MIN = 1000
    RC_STEERING_CENTER = 1500
    RC_STEERING_MAX = 2000
    RC_STEERING_DEADBAND = 50

    # ESC & Servo 輸出 (µs)
    ESC_MAX_REV = 1000
    ESC_NEUTRAL = 1518
    ESC_MAX_FWD = 2000

    # GPIO 腳位
    PPM_PIN = 23
    R_PWM_PIN = 12
    L_PWM_PIN = 13
    
    # PPM 解碼
    FRAME_SEPARATION_THRESHOLD = 4000
    MAX_CHANNELS = 6

    def __init__(self):
        self.pi = None
        self.is_running = False
        self.lock = threading.Lock()
        
        # PPM 相關變數
        self.channels = [self.RC_THROTTLE_NEUTRAL] * self.MAX_CHANNELS
        self.last_tick = 0
        self.channel_index = 0
        
        # 儲存最新的馬達指令
        self.last_left_pw = self.ESC_NEUTRAL
        self.last_right_pw = self.ESC_NEUTRAL

    def connect(self):
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print("❌ 錯誤: 無法連接到 pigpio，請先執行 'sudo pigpiod'")
                return False
            print("✅ 成功連接到 pigpio")
            return True
        except Exception as e:
            print(f"❌ 連接到 pigpio 時發生錯誤: {e}")
            return False

    def setup_hardware(self):
        print("▶ 初始化馬達硬體...")
        self.pi.set_mode(self.R_PWM_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.L_PWM_PIN, pigpio.OUTPUT)
        print(f"▶ 啟動 ESCs，發送 {self.ESC_NEUTRAL}µs 中立訊號")
        self.pi.set_servo_pulsewidth(self.R_PWM_PIN, self.ESC_NEUTRAL)
        self.pi.set_servo_pulsewidth(self.L_PWM_PIN, self.ESC_NEUTRAL)
        time.sleep(2)
        print("✅ ESCs 初始化完成。")

    def start_control_thread(self):
        self.is_running = True
        self.setup_hardware()
        self.pi.set_mode(self.PPM_PIN, pigpio.INPUT)
        self.ppm_cb = self.pi.callback(self.PPM_PIN, pigpio.FALLING_EDGE, self._ppm_callback)
        thread = threading.Thread(target=self._control_loop)
        thread.daemon = True
        thread.start()
        print("▶ 馬達控制執行緒已啟動")

    def stop_control(self):
        self.is_running = False
        time.sleep(0.1) # 等待迴圈結束
        if self.pi and self.pi.connected:
            if self.ppm_cb:
                self.ppm_cb.cancel()
            self._cleanup_hardware()
            self.pi.stop()
        print("⏹️ 馬達控制器已停止")
    
    def _cleanup_hardware(self):
        print("\n▶ 安全關閉馬達...")
        self.pi.set_servo_pulsewidth(self.R_PWM_PIN, self.ESC_NEUTRAL)
        self.pi.set_servo_pulsewidth(self.L_PWM_PIN, self.ESC_NEUTRAL)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.R_PWM_PIN, 0)
        self.pi.set_servo_pulsewidth(self.L_PWM_PIN, 0)
        print("✅ 馬達已安全關閉。")
        
    def _map_range(self, x, in_min, in_max, out_min, out_max):
        if in_max == in_min: return out_min
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _ppm_callback(self, gpio, level, tick):
        pulse_width = pigpio.tickDiff(self.last_tick, tick)
        self.last_tick = tick
        if pulse_width > self.FRAME_SEPARATION_THRESHOLD:
            self.channel_index = 0
        elif self.channel_index < self.MAX_CHANNELS:
            if 800 < pulse_width < 2200:
                with self.lock:
                    self.channels[self.channel_index] = pulse_width
            self.channel_index += 1

    def _control_loop(self):
        while self.is_running:
            with self.lock:
                steering_ch = self.channels[0]
                throttle_ch = self.channels[1]
            
            lm_pw, rm_pw = self._calculate_motor_pw(throttle_ch, steering_ch)
            
            self.pi.set_servo_pulsewidth(self.L_PWM_PIN, lm_pw)
            self.pi.set_servo_pulsewidth(self.R_PWM_PIN, rm_pw)
            
            with self.lock:
                self.last_left_pw = lm_pw
                self.last_right_pw = rm_pw
            
            time.sleep(0.02)
    
    def _calculate_motor_pw(self, throttle_ch, steering_ch):
        if abs(throttle_ch - self.RC_THROTTLE_NEUTRAL) <= self.RC_THROTTLE_DEADBAND:
            if steering_ch > (self.RC_STEERING_CENTER + self.RC_STEERING_DEADBAND):
                speed = int(self.ESC_NEUTRAL + 200)
                right_motor_pw = min(speed, self.ESC_MAX_FWD)
                left_motor_pw = self.ESC_NEUTRAL
            elif steering_ch < (self.RC_STEERING_CENTER - self.RC_STEERING_DEADBAND):
                speed = int(self.ESC_NEUTRAL + 200)
                left_motor_pw = min(speed, self.ESC_MAX_FWD)
                right_motor_pw = self.ESC_NEUTRAL
            else:
                left_motor_pw = self.ESC_NEUTRAL
                right_motor_pw = self.ESC_NEUTRAL
        else:
            if throttle_ch < self.RC_THROTTLE_NEUTRAL:
                base_throttle = self._map_range(throttle_ch, self.RC_THROTTLE_NEUTRAL - self.RC_THROTTLE_DEADBAND, self.RC_THROTTLE_MIN, self.ESC_NEUTRAL, self.ESC_MAX_FWD)
            else:
                base_throttle = self._map_range(throttle_ch, self.RC_THROTTLE_NEUTRAL + self.RC_THROTTLE_DEADBAND, self.RC_THROTTLE_MAX, self.ESC_NEUTRAL, self.ESC_MAX_REV)
            steering_effect = self._map_range(steering_ch, self.RC_STEERING_MIN, self.RC_STEERING_MAX, -300, 300)
            left_motor_pw = base_throttle + steering_effect
            right_motor_pw = base_throttle - steering_effect
            left_motor_pw = int(max(self.ESC_MAX_REV, min(left_motor_pw, self.ESC_MAX_FWD)))
            right_motor_pw = int(max(self.ESC_MAX_REV, min(right_motor_pw, self.ESC_MAX_FWD)))
        return left_motor_pw, right_motor_pw

    def get_estimated_speed(self):
        """根據目前的 PWM 指令估算船隻的前進速度"""
        with self.lock:
            left_pw = self.last_left_pw
            right_pw = self.last_right_pw

        # 使用左右馬達的平均 PWM 來估算整體推進力
        avg_pwm = (left_pw + right_pw) / 2.0
        
        speed_mps = 0.0
        if avg_pwm > self.ESC_NEUTRAL + 10: # 前進 (加入微小死區)
            speed_mps = self._map_range(avg_pwm, self.ESC_NEUTRAL, self.ESC_MAX_FWD, 0, self.MAX_FORWARD_SPEED_MPS)
        elif avg_pwm < self.ESC_NEUTRAL - 10: # 後退 (加入微小死區)
            # 後退速度為負值
            speed_mps = self._map_range(avg_pwm, self.ESC_NEUTRAL, self.ESC_MAX_REV, 0, -self.MAX_REVERSE_SPEED_MPS)
        
        return speed_mps

# --- 主導航與里程計類別 (Main Navigator & Odometry Class) ---
class AutonomousBoat:
    def __init__(self, imu_port, imu_baud, csv_path):
        self.imu = IMUProcessor(port=imu_port, baudrate=imu_baud)
        self.motor = MotorController()
        self.csv_path = csv_path
        self.x = 0.0
        self.y = 0.0
        self.last_update_time = None

    def setup_csv(self):
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'X_pos', 'Y_pos', 'Heading_Filtered', 'Heading_Raw',
                             'Estimated_Speed_mps', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 
                             'Mag_X_Calib', 'Mag_Y_Calib'])

    def start(self):
        if not self.imu.connect() or not self.motor.connect():
            print("❌ 無法連接所有硬體，程式終止。")
            return
        
        # 在啟動執行緒前執行校準
        self.imu.calibrate_gyro()

        self.imu.start_reading_thread()
        self.motor.start_control_thread()
        self.setup_csv()
        
        print("\n" + "="*50)
        print("✅ 系統準備就緒。按 Ctrl+C 停止。")
        print("="*50)
        self.last_update_time = time.time()
        
        try:
            while True:
                self.update_odometry()
                time.sleep(0.05) # 主循環更新頻率 20Hz
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("\nℹ️ 使用者中斷，正在停止系統...")
        self.imu.stop_reading()
        self.motor.stop_control()
        print(f"💾 數據收集完成並保存至 {self.csv_path}")
        self.plot_results()
        print("👋 程式結束")

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0: return
        self.last_update_time = current_time

        (heading_filtered, heading_raw,
         gyro_x, gyro_y, gyro_z,
         mag_x, mag_y) = self.imu.get_latest_data()
        
        estimated_speed_mps = self.motor.get_estimated_speed()

        # 將航向角(0=北, 90=東)轉換為數學角度(0=X正向, 90=Y正向)
        heading_rad = math.radians(90 - heading_filtered)

        vx = estimated_speed_mps * math.cos(heading_rad)
        vy = estimated_speed_mps * math.sin(heading_rad)
        
        self.x += vx * dt
        self.y += vy * dt

        sys.stdout.write(
            f"\r座標: ({self.x:6.2f}m, {self.y:6.2f}m) | "
            f"航向: {heading_filtered:5.1f}° | "
            f"估計速度: {estimated_speed_mps:4.2f} m/s "
        )
        sys.stdout.flush()

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, self.x, self.y, heading_filtered, heading_raw,
                             estimated_speed_mps, gyro_x, gyro_y, gyro_z,
                             mag_x, mag_y])
            
    def plot_results(self):
        # ... (此函數的程式碼保持不變)
        """繪製包含航向、角速度、軌跡和極座標圖的綜合圖表"""
        try:
            df = pd.read_csv(self.csv_path)
            if df.empty or len(df) < 2:
                print("CSV file has insufficient data for plotting.")
                return

            df['Time'] = df['Timestamp'] - df['Timestamp'].iloc[0]
            
            # 建立一個 2x2 的子圖網格
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle('Boat Data Analysis', fontsize=16)

            # 圖 1: 航向角隨時間變化
            ax1 = axes[0, 0]
            ax1.plot(df['Time'], df['Heading_Raw'], 'r--', alpha=0.6, label='Raw Heading')
            ax1.plot(df['Time'], df['Heading_Filtered'], 'b-', label='Filtered Heading')
            ax1.set_title('Heading Angle Over Time')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Heading (degrees)')
            ax1.grid(True)
            ax1.legend()
            ax1.set_ylim(0, 360)

            # 圖 2: 角速度隨時間變化
            ax2 = axes[0, 1]
            ax2.plot(df['Time'], df['Gyro_X'], label='Gyro X', color='red')
            ax2.plot(df['Time'], df['Gyro_Y'], label='Gyro Y', color='green')
            ax2.plot(df['Time'], df['Gyro_Z'], label='Gyro Z', color='blue')
            ax2.set_title('Angular Velocity Over Time')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (°/s)')
            ax2.grid(True)
            ax2.legend()

            # 圖 3: 里程計軌跡
            ax3 = axes[1, 0]
            ax3.plot(df['X_pos'], df['Y_pos'], 'o-', markersize=2, label='Odometry Path')
            ax3.set_title('Estimated Odometry Trajectory')
            ax3.set_xlabel('X Position (m)')
            ax3.set_ylabel('Y Position (m)')
            ax3.grid(True)
            ax3.legend()
            ax3.set_aspect('equal', adjustable='box')

            # 圖 4: 航向極座標圖
            heading_rad = np.deg2rad(df['Heading_Filtered'])
            time_radius = df['Time']
            # 使用 plt.subplot 在網格上建立極座標投影
            ax4 = plt.subplot(2, 2, 4, projection='polar')
            scatter = ax4.scatter(heading_rad, time_radius, c=time_radius, cmap='viridis', s=10)
            ax4.set_title('Heading Polar Plot (Orientation Trajectory)')
            ax4.set_theta_zero_location('N') # 北方為 0 度
            ax4.set_theta_direction(-1) # 順時針方向
            cbar = fig.colorbar(scatter, ax=ax4, pad=0.1)
            cbar.set_label('Time (s)')

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.savefig('boat_full_analysis.png', dpi=300)
            print("\n📈 全部分析圖表已保存為 boat_full_analysis.png")
            plt.show()

        except Exception as e:
            print(f"繪圖時發生錯誤: {e}")


# --- 主程式 ---
if __name__ == "__main__":
    IMU_PORT = '/dev/ttyUSB0'   # 您的 IMU 序列埠
    CSV_FILE_PATH = 'boat_motor_odometry_data.csv'

    boat = AutonomousBoat(
        imu_port=IMU_PORT,
        imu_baud=921600,
        csv_path=CSV_FILE_PATH
    )
    
    boat.start()

