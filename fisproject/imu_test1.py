import serial.tools.list_ports
import struct
import time
import crcmod.predefined
import math
import numpy as np
import csv
import pandas as pd
import os
import matplotlib.pyplot as plt
from scipy.signal import medfilt

# 設定CSV檔案路徑
csv_path = '/home/t1204/Desktop/fisproject/imu_data1.csv'

# 封包配置
df_head = 'fc'
df_end = 'fd'
addr_imu = '40'
addr_ahrs = '41'
imu_len = '38'
ahrs_len = '30'

# 串口設定
serial_port = '/dev/ttyUSB0'
serial_bps = 921600
serial_to = 3

# 磁力計校準
scale1 = [1.0037140204271124, 0.9600355239786856, 1.039423076923077]
offset1 = [-28.875, 273.75, 306.0]

# 初始化變數
initial_x = None
initial_y = None
initial_heading = None

# 卡爾曼濾波參數
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.error_estimate = 1.0
        
    def update(self, measurement):
        # 預測步驟
        prediction = self.estimate
        prediction_error = self.error_estimate + self.process_variance
        
        # 更新步驟
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.error_estimate = (1 - kalman_gain) * prediction_error
        
        return self.estimate

# 初始化卡爾曼濾波器
kf_x = KalmanFilter(process_variance=0.01, measurement_variance=1.0)
kf_y = KalmanFilter(process_variance=0.01, measurement_variance=1.0)
kf_heading = KalmanFilter(process_variance=0.01, measurement_variance=2.0)

# 中值濾波緩衝區
buffer_size = 5
x_buffer = []
y_buffer = []
heading_buffer = []

def receive_data():
    global initial_x, initial_y, initial_heading, x_buffer, y_buffer, heading_buffer

    # 創建CSV檔案並寫入表頭
    with open(csv_path, 'w', newline='') as csvfile:
        fieldnames = ['Timestamp', 'X', 'Y', 'Z', 'Heading', 'Gyro_X', 'Gyro_Y', 'Gyro_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

    try:
        while True:
            check_head = ser.read().hex()
            if check_head != df_head:
                continue
            addr = ser.read().hex()
            len_hex = ser.read().hex()
            check_sn = ser.read().hex()
            head_crc8 = ser.read().hex()
            crc16_H_s = ser.read().hex()
            crc16_L_s = ser.read().hex()

            check_headb = bytes.fromhex(check_head)
            addrb = bytes.fromhex(addr)
            lengthb = bytes.fromhex(len_hex)
            check_snb = bytes.fromhex(check_sn)

            if addr == addr_imu:
                data_s = ser.read(int(imu_len, 16))
                combined_header = check_headb + addrb + lengthb + check_snb
                crc8_calculated = crc8_maximsss(combined_header)
                crc16_calculated = calculate_crc16(data_s)
                tail = ser.read().hex()

                if (crc8_calculated == int(head_crc8, 16)):
                    # 讀取磁力計數據
                    magnetometer_x = struct.unpack('f', data_s[24:28])[0]
                    magnetometer_y = struct.unpack('f', data_s[28:32])[0]
                    magnetometer_z = struct.unpack('f', data_s[32:36])[0]

                    # 讀取陀螺儀數據
                    gyro_x = struct.unpack('f', data_s[12:16])[0]
                    gyro_y = struct.unpack('f', data_s[16:20])[0]
                    gyro_z = struct.unpack('f', data_s[20:24])[0]
                    
                    # 校準並抵消偏移
                    x = magnetometer_x * scale1[0] - offset1[0]
                    y = magnetometer_y * scale1[1] - offset1[1]
                    z = magnetometer_z * scale1[2] - offset1[2]

                    # 設定原點
                    if initial_x is None and initial_y is None:
                        initial_x = x
                        initial_y = y

                    # 計算相對位置
                    x -= initial_x
                    y -= initial_y

                    # 計算航向角
                    heading_rad = math.atan2(y, x)
                    heading_deg = math.degrees(heading_rad)
                    if heading_deg < 0:
                        heading_deg += 360
                    heading_deg = (360 - heading_deg) % 360
                    
                    # 異常值檢測 (前後差異過大時)
                    if len(x_buffer) > 0:
                        prev_x = x_buffer[-1]
                        prev_y = y_buffer[-1]
                        prev_heading = heading_buffer[-1]
                        
                        # 設定閾值，超過閾值則視為異常
                        x_threshold = 5
                        y_threshold = 5
                        heading_threshold = 30
                        
                        if abs(x - prev_x) > x_threshold:
                            x = prev_x
                        if abs(y - prev_y) > y_threshold:
                            y = prev_y
                        
                        # 針對航向角的環繞處理（0度和360度的連續性）
                        if min(abs(heading_deg - prev_heading), 
                               abs(heading_deg - prev_heading + 360), 
                               abs(heading_deg - prev_heading - 360)) > heading_threshold:
                            heading_deg = prev_heading
                    
                    # 添加到緩衝區
                    x_buffer.append(x)
                    y_buffer.append(y)
                    heading_buffer.append(heading_deg)
                    
                    # 保持緩衝區大小
                    if len(x_buffer) > buffer_size:
                        x_buffer.pop(0)
                        y_buffer.pop(0)
                        heading_buffer.pop(0)
                    
                    # 應用中值濾波
                    if len(x_buffer) >= 3:
                        x_median = np.median(x_buffer)
                        y_median = np.median(y_buffer)
                        heading_median = np.median(heading_buffer)
                    else:
                        x_median = x
                        y_median = y
                        heading_median = heading_deg
                    
                    # 應用卡爾曼濾波
                    x_filtered = kf_x.update(x_median)
                    y_filtered = kf_y.update(y_median)
                    heading_filtered = kf_heading.update(heading_median)

                    timestamp = time.time()

                    print(f"X: {x_filtered:.2f}, Y: {y_filtered:.2f}, Heading: {heading_filtered:.2f}")

                    # 寫入CSV
                    with open(csv_path, 'a', newline='') as csvfile:
                        writer = csv.DictWriter(csvfile, fieldnames=['Timestamp', 'X', 'Y', 'Z', 'Heading', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
                        writer.writerow({
                            'Timestamp': timestamp,
                            'X': x_filtered,
                            'Y': y_filtered,
                            'Z': z,
                            'Heading': heading_filtered,
                            'Gyro_X': gyro_x,
                            'Gyro_Y': gyro_y,
                            'Gyro_Z': gyro_z
                        })

            elif addr == addr_ahrs:
                ser.read(int(ahrs_len, 16))  # AHRS數據（可選）
            else:
                continue

    except KeyboardInterrupt:
        print(f"\n數據收集完成並保存至 {csv_path}")
        ser.close()
        post_process_data()  # 添加後處理步驟
        plot_results()
        exit(1)

def post_process_data():
    """收集數據後進行額外的後處理"""
    try:
        df = pd.read_csv(csv_path)
        
        # 應用窗口較大的移動平均濾波器進行額外平滑處理
        window_size = 25
        df['X_smooth'] = df['X'].rolling(window=window_size, center=True).mean().fillna(df['X'])
        df['Y_smooth'] = df['Y'].rolling(window=window_size, center=True).mean().fillna(df['Y'])
        
        # 移除路徑中的異常值
        max_change = 1.0  # 相鄰樣本間最大位置變化
        
        for i in range(1, len(df)):
            dx = df.loc[i, 'X_smooth'] - df.loc[i-1, 'X_smooth']
            dy = df.loc[i, 'Y_smooth'] - df.loc[i-1, 'Y_smooth']
            
            # 如果變化過大，則限制它
            if abs(dx) > max_change:
                df.loc[i, 'X_smooth'] = df.loc[i-1, 'X_smooth'] + np.sign(dx) * max_change
            
            if abs(dy) > max_change:
                df.loc[i, 'Y_smooth'] = df.loc[i-1, 'Y_smooth'] + np.sign(dy) * max_change
        
        # 用平滑處理後的值更新原始列
        df['X'] = df['X_smooth']
        df['Y'] = df['Y_smooth']
        
        # 保存後處理數據
        df.to_csv(csv_path, index=False)
        print("後處理完成。")
    
    except Exception as e:
        print(f"後處理過程中發生錯誤: {e}")

def crc8_maximsss(data):
    crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
    return crc8_func(data)

def calculate_crc16(data):
    crc16_func = crcmod.predefined.mkPredefinedCrcFun('crc-16')
    return crc16_func(data)

def plot_results():
    df = pd.read_csv(csv_path)

    # 創建子圖顯示路徑和航向
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    
    # 繪製路徑圖 (X vs Y)
    axes[0].plot(df['X'], df['Y'], color='blue', label='Path')
    axes[0].set_title('Path')
    axes[0].set_xlabel('X Coordinate')
    axes[0].set_ylabel('Y Coordinate')
    axes[0].grid(True)
    axes[0].axis('equal')
    axes[0].legend()
    
    # 繪製航向角隨時間變化圖
    axes[1].plot(df['Timestamp'] - df['Timestamp'].iloc[0], df['Heading'], color='blue', label='Heading Angle')
    axes[1].set_title('Heading Angle Over Time')
    axes[1].set_xlabel('Time')
    axes[1].set_ylabel('Heading Angle (degrees)')
    axes[1].grid(True)
    axes[1].legend()
    
    # 調整佈局並保存
    plt.tight_layout()
    plt.savefig('/home/t1204/Desktop/fisproject/combined_plots.png')
    plt.show()

if __name__ == "__main__":
    try:
        ser = serial.Serial(
            port=serial_port, baudrate=serial_bps, timeout=serial_to)
    except:
        print("Error: unable to open serial port")
        exit(1)

    receive_data()
