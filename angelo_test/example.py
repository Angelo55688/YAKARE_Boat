import serial
import struct
import time
import crcmod.predefined
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv

# --- 卡爾曼濾波器類別 ---
class KalmanFilter:
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

# --- IMU 處理器類別 ---
class IMUProcessor:
    def __init__(self, port, baudrate, csv_path):
        # 串口配置
        self.serial_port = port
        self.serial_bps = baudrate
        self.ser = None
        self.csv_path = csv_path

        # 封包配置
        self.df_head = b'\xfc'
        self.addr_imu = 0x40
        self.imu_len = 0x38

        # 磁力計校準參數 (請務必使用您自己的校準值)
        self.mag_scale = [1.0037, 0.9600, 1.0394]
        self.mag_offset = [-28.875, 273.75, 306.0]

        # 初始化變數
        self.initial_heading_offset = None

        # 濾波器
        self.kf_heading = KalmanFilter(process_variance=0.01, measurement_variance=2.0)
        self.heading_buffer = []
        self.buffer_size = 5

        # 初始化 CRC 函數
        self.crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
        self.crc16_func = crcmod.predefined.mkPredefinedCrcFun('crc-16')

    def connect(self):
        """連接到序列埠"""
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_bps, timeout=1)
            print(f"成功連接到 {self.serial_port}")
            return True
        except serial.SerialException as e:
            print(f"錯誤: 無法開啟序列埠 {self.serial_port}. {e}")
            return False

    def setup_csv(self):
        """創建 CSV 檔案並寫入表頭"""
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Heading_Raw', 'Heading_Filtered', 
                             'Gyro_X', 'Gyro_Y', 'Gyro_Z', 
                             'Mag_X_Calib', 'Mag_Y_Calib'])

    def receive_and_process(self):
        """主循環，讀取並處理數據"""
        try:
            while True:
                if self.ser.read(1) != self.df_head:
                    continue
                header_data = self.ser.read(6)
                if len(header_data) < 6:
                    continue
                addr, length, sn, head_crc8, crc16_H, crc16_L = struct.unpack('<BBBBBB', header_data)
                header_to_check = self.df_head + header_data[:3]
                if self.crc8_func(header_to_check) != head_crc8:
                    continue
                if addr == self.addr_imu and length == self.imu_len:
                    payload = self.ser.read(length)
                    self.ser.read(1) # 讀取並丟棄封包尾
                    if len(payload) == length:
                        self.parse_imu_payload(payload)
        except KeyboardInterrupt:
            print(f"\n數據收集完成並保存至 {self.csv_path}")
            if self.ser:
                self.ser.close()
            self.plot_results()

    def parse_imu_payload(self, data):
        """解析 IMU 數據負載"""
        gyro_x, gyro_y, gyro_z = struct.unpack_from('<fff', data, 12)
        mag_x, mag_y, mag_z = struct.unpack_from('<fff', data, 24)

        calib_x = (mag_x - self.mag_offset[0]) * self.mag_scale[0]
        calib_y = (mag_y - self.mag_offset[1]) * self.mag_scale[1]

        heading_rad = math.atan2(-calib_y, calib_x)
        heading_deg = math.degrees(heading_rad)
        heading_deg = (heading_deg + 360) % 360

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
            
        heading_filtered = self.kf_heading.update(heading_median)
        heading_filtered = heading_filtered % 360

        timestamp = time.time()
        print(f"原始航向: {heading_raw:6.1f}°,  濾波後航向: {heading_filtered:6.1f}°")

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, heading_raw, heading_filtered, 
                             math.degrees(gyro_x), math.degrees(gyro_y), math.degrees(gyro_z),
                             calib_x, calib_y])

    def plot_results(self):
        """繪製結果圖表，包含航向、角速度、羅盤軌跡和航向極座標圖"""
        try:
            df = pd.read_csv(self.csv_path)
            if df.empty:
                print("CSV 檔案為空，無法繪圖。")
                return

            df['Time'] = df['Timestamp'] - df['Timestamp'].iloc[0]
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle('IMU Data Analysis', fontsize=16)

            ax1 = axes[0, 0]
            ax1.plot(df['Time'], df['Heading_Raw'], 'r--', alpha=0.6, label='原始航向 (Raw Heading)')
            ax1.plot(df['Time'], df['Heading_Filtered'], 'b-', label='濾波後航向 (Filtered Heading)')
            ax1.set_title('Heading Angle Over Time')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Heading (degrees)')
            ax1.grid(True)
            ax1.legend()
            ax1.set_ylim(0, 360)

            ax2 = axes[0, 1]
            ax2.plot(df['Time'], df['Gyro_X'], label='Gyro X', color='red')
            ax2.plot(df['Time'], df['Gyro_Y'], label='Gyro Y', color='green')
            ax2.plot(df['Time'], df['Gyro_Z'], label='Gyro Z', color='blue')
            ax2.set_title('Angular Velocity Over Time')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (°/s)')
            ax2.grid(True)
            ax2.legend()

            ax3 = axes[1, 0]
            ax3.plot(df['Mag_X_Calib'], df['Mag_Y_Calib'], 'o-', markersize=2, label='Magnetometer Path')
            ax3.set_title('Calibrated Magnetometer Trajectory (Compass)')
            ax3.set_xlabel('Mag X Calibrated')
            ax3.set_ylabel('Mag Y Calibrated')
            ax3.grid(True)
            ax3.legend()
            ax3.set_aspect('equal', adjustable='box')

            heading_rad = np.deg2rad(df['Heading_Filtered'])
            time_radius = df['Time']
            ax4 = plt.subplot(2, 2, 4, projection='polar')
            scatter = ax4.scatter(heading_rad, time_radius, c=time_radius, cmap='viridis', s=10)
            ax4.set_title('Heading Polar Plot (Orientation Trajectory)')
            ax4.set_theta_zero_location('N')
            ax4.set_theta_direction(-1)
            cbar = fig.colorbar(scatter, ax=ax4, pad=0.1)
            cbar.set_label('Time (s)')

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.savefig('imu_full_analysis.png', dpi=300)
            plt.show()

        except Exception as e:
            print(f"繪圖時發生錯誤: {e}")

# --- 主程式 ---
if __name__ == "__main__":
    csv_file_path = 'imu_heading_data.csv'
    processor = IMUProcessor(port='/dev/ttyUSB0', baudrate=921600, csv_path=csv_file_path)

    if processor.connect():
        processor.setup_csv()
        processor.receive_and_process()