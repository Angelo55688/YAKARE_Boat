import serial
import time
import re  # For extracting numbers

# Open serial connection (adjust the port accordingly)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def read_data():
    """
    Improved data reading function with better error handling
    Returns None if data is invalid instead of default zeros
    """
    try:
        # Read until we get a complete line
        data = ser.readline().decode('utf-8').strip()
        
        # Skip empty lines
        if not data:
            print("Received empty line")
            return None
            
        # Debug raw data
        print(f"Raw data received: {data}")
        
        # Split and convert to floats
        try:
            distances = [float(x) for x in data.split(',')]
        except ValueError as e:
            print(f"Data conversion failed: {e} - Data: {data}")
            return None
            
        # Validate we got exactly 3 distances
        if len(distances) != 3:
            print(f"Expected 3 distances, got {len(distances)}")
            return None
            
        return distances
        
    except Exception as e:
        print(f"Serial communication error: {e}")
        return None
  # Main loop
try:
    print("Waiting for Arduino data...")
    while True:
        distances = read_data()
        
        if distances is None:
            time.sleep(0.1)  # Short delay before retrying
            continue
            
        distance_right, distance_center, distance_left = distances
        min_distance = min(distances)
        
        # Determine obstacle position
        if min_distance == distance_right:
            yaw = -30
        elif min_distance == distance_center:
            yaw = 0
        else:
            yaw = 30

        print(f"Valid data - Distance: {min_distance}, Yaw: {yaw}")
        print(f"Details - L: {distance_left:.1f}, C: {distance_center:.1f}, R: {distance_right:.1f}")
        
        # Add your FIS processing here

except KeyboardInterrupt:
    print("\nStopping program...")

finally:
    ser.close()
    print("Serial connection closed.")