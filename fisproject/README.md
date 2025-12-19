# Fisproject
This sub-repo contains the conference code for the project YAKARE Boat.

## Prerequisites
Make sure that you have all the prerequisites installed described in the main README.md file. Instructions can be found here [```README.md```](./../README.md).

**Additionally**, it is important that the local python environment is instantiated for the programs to work as intended. This is done by running this command in a terminal in the local directory:
```bash
source venv/bin/activate
```

## Usage
The project architecture is designed to bridge high-level computer vision with low-level hardware control on a Raspberry Pi.

### **fis.py**
The [```fis.py```](./fis.py) is the main script used in this folder. It implements a Fuzzy Inference System (FIS) for autonomous obstacle avoidance. It utilizes the skfuzzy library to define membership functions for distance (Close, Med, Far) and yaw (Port, Front, Starboard) to compute the appropriate PWM speeds for the left and right motors. A Zero-Order Hold (ZOH) mechanism is implemented to maintain the last valid sensor reading in the event of serial data dropouts, ensuring continuous motor control.

Execution: Run python fis.py.

Input: Tri-directional distance data via serial and fuzzy logic rules defined within the script.

Output: Hardware PWM signals sent to R_PWM_PIN (GPIO 12) and L_PWM_PIN (GPIO 13) to drive the vehicle's motors.

### **sensor_data.py**
This script is used for debugging and testing the sonar sensor. It establishes a serial connection via */dev/ttyACM0* to receive a stream of three distance values representing the left, center, and right proximity sensors. The script calculates the minimum distance to identify the nearest obstacle and assigns a corresponding yaw angle to indicate the obstacle's relative position.

**Execution**
```bash
python sensor_data.py.
```

**Input**
Raw CSV-formatted strings (e.g., "150.5, 200.0, 50.2") via serial communication.

**Output**
Real-time terminal logs displaying validated distance readings and the calculated yaw position.

### **imu_test1.py**
The [```imu_test1.py```](./imu_test1.py) script is a comprehensive utility for capturing, filtering, and visualizing 6-DOF (Degrees of Freedom) inertial data from an IMU sensor via a high-speed serial connection. It specifically targets magnetometer and gyroscope data to calculate a device's heading and relative planar movement. To ensure data integrity, the script employs a multi-stage signal processing pipeline:

**Execution**
```bash
python imu_test1.py.
```

**Input**
High-frequency binary data packets (921600 bps) from a serial device at */dev/ttyUSB0*.

**Data Log**
Processed and smoothed results are saved to *imu_data1.csv*.

**Visualization**
Generates a two-panel diagnostic image saved as *combined_plots.png*, showing the calculated 2D path and the heading angle over time.

## Contributing
The project does not currently accept outside contributions.

## License
APACHE 2.0