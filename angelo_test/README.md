# Angelo Test
This sub-repo contains the testing and demonstration code for the project YAKARE Boat.

## Prerequisites
The prerequisite libraries used are installed using the installation command further down. A list of the required libraries and their versions can be found in [```requirements.txt```](./requirements.txt)

## Installation
To install the required libraries, run this bash command:
```bash
pip install -r requirements.txt
```

## Usage
The project architecture is designed to bridge high-level computer vision with low-level hardware control on a Raspberry Pi.

### **test0813.py**
The [```test0813.py```](./test0813.py) script serves as a hardware validation and manual control utility for the Raspberry Pi vehicle. It focuses on decoding PPM (Pulse Position Modulation) signals from a radio receiver to drive the vehicle's propulsion and auxiliary systems. The code implements a differential steering logic that translates remote control stick inputs into microsecond pulse-width modulation (PWM) signals for dual Electronic Speed Controllers (ESCs). Additionally, it maps a specific auxiliary channel to control a trigger servo motor.

**Execution**
```bash
python test0813.py.
```

**Input**
Real-time PPM signals via *PPM_PIN* (GPIO 23) from an RC receiver.

**Output**
Real-time terminal telemetry showing channel values and motor pulse widths. It drives physical outputs on *L_PWM_PIN* (GPIO 13), *R_PWM_PIN* (GPIO 12), and *TRIGGER_PIN* (GPIO 24).

** Functional Overview
1. Signal Processing (PPM Decoding)
The script uses a callback function on the *PPM_PIN* to measure the timing between falling edges of the incoming signal. It distinguishes between individual channels and frame gaps to populate a channels array with precise microsecond values representing the user's stick positions.

2. Control Logic (Differential Steering)
The navigation logic processes two primary inputs:
+ Throttle: Determines base speed for both motors, including a "deadband" to prevent creeping when the stick is centered.
+ Steering: Calculates a steering_effect that is added to one motor and subtracted from the other, enabling the vehicle to turn by varying the speeds of the left and right wheels.

3. **Hardware Execution (PWM)**
The pigpio library generates stable PWM hardware signals to control the connected actuators.
+ ESCs: Receive signals ranging from *ESC_MAX_REV* (1000µs) to *ESC_MAX_FWD* (2000µs) to control motor speed and direction.

+ Trigger Servo: Translates *CH5* inputs into specific angular positions for mechanical actuation.

### **example.py**
The [```example.py```](./example.py) script facilitates real-time IMU data acquisition, processing, and visualization. It connects to an IMU sensor via a serial interface to parse gyroscope and magnetometer data, applying a Kalman Filter and median filtering to ensure high-accuracy heading calculations.

**Execution**
To start data collection and processing, run the script from your terminal:
```bash
python example.py
```

**Input and Output**
Input: The script listens for raw data packets from a serial device, typically mapped to ```/dev/ttyUSB0```.

Data Log: Processed information, including filtered headings and angular velocity, is saved to [```imu_heading_data.csv```](./imu_heading_data.csv).

Visualization: Upon stopping the script (**Ctrl+C**), a comprehensive analysis image is generated and saved to ```imu_full_analysis.png``` in the workspace directory.

### **main.py** (INCOMPLETE)
The target following capability is still not yet implemented. The files [```example.py```](./example.py) and [```test0813.py```](./test0813.py) are used as the working and testing code.

The [```main.py```](./main.py) script serves as an all-in-one control system for a Raspberry Pi-based vehicle featuring autonomous visual tracking and a web-based monitoring interface. It integrates hardware PWM control via the pigpio library to manage Electronic Speed Controllers (ESCs), steering servos, and a trigger mechanism. The system decodes PPM (Pulse Position Modulation) signals for manual RC override while simultaneously utilizing OpenCV to detect and track red-colored targets. A built-in PID controller processes the visual error to automate steering.

**Execution**
```bash
python main.py.
```

**Input**
Real-time video from the primary camera and PPM signals via PPM_PIN (**GPIO 23**).

**Output**
Live web stream accessible via browser at *http://<IP_ADDRESS>:8080*.

### **server.py** (INCOMPLETE)
This script provides a dedicated backend implementation designed for high-performance remote video streaming and hardware control via a TCP/IP socket connection. It is intended to run on the Raspberry Pi as a server that waits for a specific client connection on **port 8485** rather than hosting a general web page. It optimizes the delivery of the video feed by transmitting raw, encoded JPEG frames prefixed with size headers.

**Execution**
```bash
python server.py.
```

**Input**
Raw camera feed and incoming socket connection requests.

**Output**
High-speed binary stream of JPEG frames and telemetry data to a connected socket client.

## Contributing
The project does not currently accept outside contributions.

## License
APACHE 2.0