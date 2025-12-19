# **YAKARE_Boat**
This project serves as a backup for the YAKARE boat project so that it can be rebuilt if it is destroyed during testing.

## About the Project
The projects goal is to implement remote and autonomous control of an RC boat. It is for both research and educational purposes for students at NTUST.

## Getting Started
This repository contains two separate sub projects. The folder [```angelo_test```](./angelo_test) is for testing and demonstrations, while the folder [```fisproject```](./fisproject) is for conferences. For further details on the contents of each folder, see their respective README.md file.

To clone the repository, run the following bash command in a terminal of your choice:
```bash
git clone "https://github.com/Angelo55688/YAKARE_Boat.git"
cd YAKARE_Boat
```

## Prerequisites
The code was built and tested using Python 3.11.2, so functionality cannot be guaranteed using other versions. This is true for both subfolders. Make sure to also install the required python packages using their respective requirements.txt files.

## Installation
See the sub folders README.md files for installation instructions.

New requirements.txt files can be generated using pipreqs. Execute the following bash commands in a terminal **in the local folder of the .py files**:
```bash
pip install pipreqs
pipreqs .
```

## Usage
In order for pigpio to function normally, the pigpio daemon needs to be running on the Raspberry Pi.
To run the pigpio daemon, simply run the following bash command:
```bash
sudo pigpio
```

## Hardware List
The hardware used in the project is described below. For more information, see the document [```README.pptx```](./README.pptx)
+Camera - C270
+Ultrasonic Sensor - HC-SR04
+IMU - N100
+Microcontroller - Arduino UNO
+Main Processor - Raspberry Pi 4
+Battery - 5200mAh4S
+Weapon System - Action Army AAP-01
+Brushless ESC - XRotor 40A
+Brushless Motor - SunnySky V2216
+Servo Motor - MG996R
+Mounting Frame - 3D Printed Bracket

## Contributing
The project does not currently accept outside contributions.

## License
The project has the APACHE 2.0 license described in [```LICENSE```](./LICENSE.txt)