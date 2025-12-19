import serial
import struct
import csv
import time
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt

# Configuration
CSV_PATH = '/home/t1204/Desktop/angelo_test/imu_data.csv'
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 921600
TIMEOUT = 3

# Protocol Constants
HEADER = 'fc'
FOOTER = 'fd'
ADDR_IMU = '40'
IMU_DATA_LENGTH = 56  # 0x38 bytes

# Calibration Parameters
ACCEL_SCALE = [1.0, 1.0, 1.0]  # Accelerometer scale factors
GYRO_SCALE = [1.0, 1.0, 1.0]   # Gyroscope scale factors

def butter_lowpass_filter(data, cutoff, fs, order=5):
    """Apply a low-pass Butterworth filter to data"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def validate_data(value, min_val, max_val, default=0):
    """Validate data is within reasonable bounds"""
    if np.isnan(value) or value < min_val or value > max_val:
        return default
    return value

def calibrate_imu(serial_port, samples=500):
    """Perform IMU calibration to compute bias values"""
    print("正在進行IMU校準... 請確保設備保持靜止")
    
    accel_samples = {'x': [], 'y': [], 'z': []}
    gyro_samples = {'x': [], 'y': [], 'z': []}
    
    for i in range(samples):
        # Read protocol header
        while True:
            header = serial_port.read().hex()
            if header == HEADER:
                break
        
        addr = serial_port.read().hex()
        if addr != ADDR_IMU:
            continue
        
        data_length = int(serial_port.read().hex(), 16)
        sn = serial_port.read().hex()
        crc8_header = serial_port.read().hex()
        
        # Read data section
        data = serial_port.read(data_length)
        crc16 = serial_port.read(2)
        footer = serial_port.read().hex()
        
        # Parse raw data
        ax, ay, az, gx, gy, gz = parse_imu_data_raw(data)
        
        # Check if values are within reasonable range
        if abs(ax) < 100 and abs(ay) < 100 and abs(az) < 100:
            accel_samples['x'].append(ax)
            accel_samples['y'].append(ay)
            accel_samples['z'].append(az)
        
        if abs(gx) < 10 and abs(gy) < 10 and abs(gz) < 10:
            gyro_samples['x'].append(gx)
            gyro_samples['y'].append(gy)
            gyro_samples['z'].append(gz)
        
        if i % 50 == 0:
            print(f"校準進度: {i}/{samples}")
    
    # Check if we have enough valid samples
    min_samples = samples * 0.5  # At least 50% of samples should be valid
    for axis in ['x', 'y', 'z']:
        if len(accel_samples[axis]) < min_samples or len(gyro_samples[axis]) < min_samples:
            print(f"警告: 軸 {axis} 的有效樣本不足。校準可能不準確。")
    
    # Calculate bias values
    accel_bias = [
        np.mean(accel_samples['x']) if accel_samples['x'] else 0,
        np.mean(accel_samples['y']) if accel_samples['y'] else 0,
        np.mean(accel_samples['z']) - 9.81 if accel_samples['z'] else -9.81  # Subtract gravity from z-axis
    ]
    
    gyro_bias = [
        np.mean(gyro_samples['x']) if gyro_samples['x'] else 0,
        np.mean(gyro_samples['y']) if gyro_samples['y'] else 0,
        np.mean(gyro_samples['z']) if gyro_samples['z'] else 0
    ]
    
    print("校準完成!")
    print(f"加速度計偏移: {accel_bias}")
    print(f"陀螺儀偏移: {gyro_bias}")
    
    return accel_bias, gyro_bias

def parse_imu_data_raw(data):
    """Parse IMU binary data packet without applying calibration"""
    accel_x = struct.unpack('<f', data[0:4])[0]
    accel_y = struct.unpack('<f', data[4:8])[0]
    accel_z = struct.unpack('<f', data[8:12])[0]
    
    gyro_x = struct.unpack('<f', data[12:16])[0]
    gyro_y = struct.unpack('<f', data[16:20])[0]
    gyro_z = struct.unpack('<f', data[20:24])[0]
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def parse_imu_data(data):
    """Original parse function (kept for compatibility)"""
    accel_x = struct.unpack('<f', data[0:4])[0] * ACCEL_SCALE[0]
    accel_y = struct.unpack('<f', data[4:8])[0] * ACCEL_SCALE[1]
    accel_z = struct.unpack('<f', data[8:12])[0] * ACCEL_SCALE[2]
    
    gyro_x = struct.unpack('<f', data[12:16])[0] * GYRO_SCALE[0]
    gyro_y = struct.unpack('<f', data[16:20])[0] * GYRO_SCALE[1]
    gyro_z = struct.unpack('<f', data[20:24])[0] * GYRO_SCALE[2]
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def parse_imu_data_calibrated(data, accel_bias, gyro_bias):
    """Parse and calibrate IMU data with bias correction and validation"""
    # Extract raw values
    accel_x = struct.unpack('<f', data[0:4])[0]
    accel_y = struct.unpack('<f', data[4:8])[0]
    accel_z = struct.unpack('<f', data[8:12])[0]
    
    gyro_x = struct.unpack('<f', data[12:16])[0]
    gyro_y = struct.unpack('<f', data[16:20])[0]
    gyro_z = struct.unpack('<f', data[20:24])[0]
    
    # Validate and apply calibration
    accel_x = validate_data(accel_x, -100, 100, 0) - accel_bias[0]
    accel_y = validate_data(accel_y, -100, 100, 0) - accel_bias[1]
    accel_z = validate_data(accel_z, -100, 100, 0) - accel_bias[2]
    
    gyro_x = validate_data(gyro_x, -10, 10, 0) - gyro_bias[0]
    gyro_y = validate_data(gyro_y, -10, 10, 0) - gyro_bias[1]
    gyro_z = validate_data(gyro_z, -10, 10, 0) - gyro_bias[2]
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def save_to_csv(data, writer):
    """Save data to CSV"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    writer.writerow([
        timestamp,
        data[0], data[1], data[2],  # Acceleration
        data[3], data[4], data[5]   # Gyroscope
    ])

def plot_data():
    """Plot acceleration, gyroscope, trajectory, and heading angle"""
    df = pd.read_csv(CSV_PATH)
    
    # Time series processing
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    time_diff = df['timestamp'].diff().dt.total_seconds().fillna(0)
    time_elapsed = np.cumsum(time_diff)
    
    # Calculate sampling frequency
    fs = 1.0 / np.mean(time_diff[1:]) if len(time_diff) > 1 else 100
    
    # Apply low-pass filter
    cutoff = 2.0  # 2 Hz cutoff frequency
    
    # Filter data
    df['accel_x_filtered'] = butter_lowpass_filter(df['accel_x'], cutoff, fs)
    df['accel_y_filtered'] = butter_lowpass_filter(df['accel_y'], cutoff, fs)
    df['gyro_z_filtered'] = butter_lowpass_filter(df['gyro_z'], cutoff, fs)
    
    # Improved trajectory calculation
    velocity_x = np.zeros_like(df['accel_x_filtered'])
    velocity_y = np.zeros_like(df['accel_y_filtered'])
    position_x = np.zeros_like(df['accel_x_filtered'])
    position_y = np.zeros_like(df['accel_y_filtered'])
    
    # Apply drift compensation
    decay_factor = 0.98  # Slight decay to prevent drift
    
    for i in range(1, len(df)):
        dt = time_diff[i]
        
        # Apply velocity calculation with decay
        velocity_x[i] = decay_factor * velocity_x[i-1] + df['accel_x_filtered'][i] * dt
        velocity_y[i] = decay_factor * velocity_y[i-1] + df['accel_y_filtered'][i] * dt
        
        # Calculate positions
        position_x[i] = position_x[i-1] + velocity_x[i] * dt
        position_y[i] = position_y[i-1] + velocity_y[i] * dt
    
    # Heading calculation with filtered data
    heading_rad = np.cumsum(df['gyro_z_filtered'] * time_diff)
    heading_deg = np.degrees(heading_rad) % 360
    
    # Create plot
    plt.figure(figsize=(12, 16))
    
    # Acceleration plot
    plt.subplot(4, 1, 1)
    plt.plot(time_elapsed, df['accel_x_filtered'], label='X-axis')
    plt.plot(time_elapsed, df['accel_y_filtered'], label='Y-axis')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True)
    plt.title('Filtered Acceleration Data')

    # Gyroscope plot
    plt.subplot(4, 1, 2)
    plt.plot(time_elapsed, df['gyro_x'], label='X-axis')
    plt.plot(time_elapsed, df['gyro_y'], label='Y-axis')
    plt.plot(time_elapsed, df['gyro_z_filtered'], label='Z-axis (filtered)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.title('Gyroscope Data')

    # Trajectory plot
    plt.subplot(4, 1, 3)
    plt.plot(position_x, position_y, color='darkorange')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.title('Estimated Trajectory (XY Plane)')
    plt.axis('equal')  # Equal scaling of axes

    # Heading angle plot
    plt.subplot(4, 1, 4)
    plt.plot(time_elapsed, heading_deg, color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('Heading Angle (°)')
    plt.yticks(np.arange(0, 361, 45))
    plt.grid(True)
    plt.title('Heading Angle Change Over Time')

    plt.tight_layout()
    plt.savefig('imu_analysis_filtered.png', dpi=300)
    plt.show()

def main():
    # Initialize CSV file
    with open(CSV_PATH, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp','accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z'])
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"已連接到 {SERIAL_PORT}")
        
        # Perform calibration
        print("開始IMU校準程序...")
        accel_bias, gyro_bias = calibrate_imu(ser)
        
        # Wait for user confirmation
        input("校準完成。按Enter鍵開始數據收集...")
        
        with open(CSV_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            
            while True:
                # Read protocol header
                header = ser.read().hex()
                if header != HEADER:
                    continue
                
                # Read subsequent protocol fields
                addr = ser.read().hex()
                if addr != ADDR_IMU:
                    continue
                
                data_length = int(ser.read().hex(), 16)
                sn = ser.read().hex()
                crc8_header = ser.read().hex()
                
                # Read data section
                data = ser.read(data_length)
                crc16 = ser.read(2)
                footer = ser.read().hex()
                
                # Parse data with calibration
                accel_gyro_data = parse_imu_data_calibrated(data, accel_bias, gyro_bias)
                save_to_csv(accel_gyro_data, writer)
                
                # Print values with better formatting
                print(f"加速度: [{accel_gyro_data[0]:.2f}, {accel_gyro_data[1]:.2f}, {accel_gyro_data[2]:.2f}] | "
                      f"陀螺儀: [{accel_gyro_data[3]:.2f}, {accel_gyro_data[4]:.2f}, {accel_gyro_data[5]:.2f}]")
                
    except serial.SerialException as e:
        print(f"串口錯誤: {e}")
    except KeyboardInterrupt:
        print("\n數據收集完成")
        plot_data()
        print("分析結果已保存為 imu_analysis_filtered.png")
    finally:
        # 確保串口正確關閉
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已關閉")
        
if __name__ == "__main__":
    main()
