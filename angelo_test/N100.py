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

# Set path for CSV file
csv_path = '/home/t1204/Desktop/angelo_test/imu_data4.csv'

# Packet configuration
df_head = 'fc'
df_end = 'fd'
addr_imu = '40'
addr_ahrs = '41'
imu_len = '38'
ahrs_len = '30'

# Serial port settings
serial_port = '/dev/ttyUSB0'
serial_bps = 921600
serial_to = 3

# Magnetometer calibration
scale1 = [1.0037140204271124, 0.9600355239786856, 1.039423076923077]
offset1 = [-28.875, 273.75, 306.0]

# To offset X/Y coordinates from the starting point
initial_x = None
initial_y = None

def receive_data():
    global initial_x, initial_y

    # Create CSV file and write header
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
                    # Read magnetometer data
                    magnetometer_x = struct.unpack('f', data_s[24:28])[0]
                    magnetometer_y = struct.unpack('f', data_s[28:32])[0]
                    magnetometer_z = struct.unpack('f', data_s[32:36])[0]

                    # Calibration and offset
                    x = magnetometer_x * scale1[0] - offset1[0]
                    y = magnetometer_y * scale1[1] - offset1[1]
                    z = magnetometer_z * scale1[2] - offset1[2]

                    # Use initial position as origin
                    if initial_x is None and initial_y is None:
                        initial_x = x
                        initial_y = y

                    x -= initial_x
                    y -= initial_y

                    # Compute heading angle
                    heading_rad = math.atan2(y, x)
                    heading_deg = math.degrees(heading_rad)
                    if heading_deg < 0:
                        heading_deg += 360
                    heading_deg = (360 - heading_deg) % 360

                    # Gyroscope data
                    gyro_x = struct.unpack('f', data_s[12:16])[0]
                    gyro_y = struct.unpack('f', data_s[16:20])[0]
                    gyro_z = struct.unpack('f', data_s[20:24])[0]

                    timestamp = time.time()

                    print(f"X: {x:.2f}, Y: {y:.2f}, Heading: {heading_deg:.2f}")

                    # Write to CSV
                    with open(csv_path, 'a', newline='') as csvfile:
                        writer = csv.DictWriter(csvfile, fieldnames=['Timestamp', 'X', 'Y', 'Z', 'Heading', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
                        writer.writerow({
                            'Timestamp': timestamp,
                            'X': x,
                            'Y': y,
                            'Z': z,
                            'Heading': heading_deg,
                            'Gyro_X': gyro_x,
                            'Gyro_Y': gyro_y,
                            'Gyro_Z': gyro_z
                        })

            elif addr == addr_ahrs:
                ser.read(int(ahrs_len, 16))  # AHRS data (optional)
            else:
                continue

    except KeyboardInterrupt:
        print(f"\nData collection completed and saved to {csv_path}")
        ser.close()
        plot_results()
        exit(1)

def crc8_maximsss(data):
    crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
    return crc8_func(data)

def calculate_crc16(data):
    crc16_func = crcmod.predefined.mkPredefinedCrcFun('crc-16')
    return crc16_func(data)

def plot_results():
    df = pd.read_csv(csv_path)

    # Plot path (X vs Y)
    plt.figure(figsize=(8, 6))
    plt.plot(df['X'], df['Y'], linestyle='-', color='blue', label='Path')
    plt.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', s=100, label='Start')
    plt.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', s=100, label='End')
    plt.text(df['X'].iloc[0], df['Y'].iloc[0], 'Start', fontsize=10, color='green', verticalalignment='bottom')
    plt.text(df['X'].iloc[-1], df['Y'].iloc[-1], 'End', fontsize=10, color='red', verticalalignment='top')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('IMU Path (X vs Y)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.savefig('/home/t1204/Desktop/angelo_test/path_plot.png')
    plt.show()

    # Plot angular velocity
    plt.figure(figsize=(10, 6))
    plt.plot(df['Timestamp'], df['Gyro_X'], label='Gyro_X')
    plt.plot(df['Timestamp'], df['Gyro_Y'], label='Gyro_Y')
    plt.plot(df['Timestamp'], df['Gyro_Z'], label='Gyro_Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (deg/s)')
    plt.title('Angular Velocity vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('/home/t1204/Desktop/angelo_test/gyro_plot.png')
    plt.show()

if __name__ == "__main__":
    try:
        ser = serial.Serial(
            port=serial_port, baudrate=serial_bps, timeout=serial_to)
    except:
        print("Error: unable to open serial port")
        exit(1)

    receive_data()
