import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import serial
import time
import pigpio

# Hardware Configuration
R_PWM_PIN = 12  # Right motor GPIO pin
L_PWM_PIN = 13  # Left motor GPIO pin

# ESC Parameters
MIN_PWM = 1000  # ESC arm/disarm value
MAX_PWM = 2000  # Maximum throttle value

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

# Function to set PWM for the motors
def set_throttle(pin, value):
    """Constrain and set pulse width for ESC"""
    constrained = max(MIN_PWM, min(value, MAX_PWM))
    pi.set_servo_pulsewidth(pin, constrained)

# Arm ESCs
print("Arming ESCs...")
set_throttle(R_PWM_PIN, MIN_PWM)
set_throttle(L_PWM_PIN, MIN_PWM)
time.sleep(2)


# Open serial connection (adjust the port accordingly)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# ZOH Variables
last_valid_data = {'distance': 1500, 'yaw': 0}  # Initial safe values
TS = 1  # Sampling time

def read_data_zoh():
    """Read data with ZOH fallback"""
    try:
        data = ser.readline().decode('utf-8').strip()
        if not data:
            return None
            
        distances = list(map(float, data.split(',')))
        if len(distances) != 3:
            return None
            
        return distances
    except:
        return None
    


# Define fuzzy variables (Inputs)
distance = ctrl.Antecedent(np.arange(0, 2001, 1), 'Distance')
yaw_angle = ctrl.Antecedent(np.arange(-45, 46, 1), 'Yaw_angle')

# Define fuzzy variables (Outputs)
right_motor = ctrl.Consequent(np.arange(1000, 1070+1, 1), 'Right_Motor_Speed')
left_motor = ctrl.Consequent(np.arange(1000, 1070+1, 1), 'Left_Motor_Speed')

# Membership functions for Distance
distance['close'] = fuzz.trimf(distance.universe, [0, 500, 1000])
distance['med'] = fuzz.trimf(distance.universe, [1000, 1500, 1600])
distance['far'] = fuzz.trimf(distance.universe, [1600, 1800, 2001])

# Membership functions for Yaw Angle
yaw_angle['port'] = fuzz.trimf(yaw_angle.universe, [-40, -30, -10])
yaw_angle['front'] = fuzz.trimf(yaw_angle.universe, [-10, 0, 10])
yaw_angle['starboard'] = fuzz.trimf(yaw_angle.universe, [10, 30, 40])

# Membership functions for motor PWM outputs
for motor in [right_motor, left_motor]:
    motor['stop'] = fuzz.trapmf(motor.universe, [1000, 1020, 1030, 1040])
    motor['slow'] = fuzz.trapmf(motor.universe, [1040, 1050, 1060, 1070])
    motor['fast'] = fuzz.trapmf(motor.universe, [1060, 1070, 1080, 1090])

# Define rules
# Modified fuzzy rules for direct PWM control
rules = [
    # Obstacle close on port
    ctrl.Rule(distance['close'] & yaw_angle['port'], 
             (right_motor['stop'], left_motor['fast'])),
    
    # Obstacle close ahead
    ctrl.Rule(distance['close'] & yaw_angle['front'], 
             (right_motor['slow'], left_motor['slow'])),
    
    # Obstacle close on starboard
    ctrl.Rule(distance['close'] & yaw_angle['starboard'], 
             (right_motor['fast'], left_motor['stop'])),
    
    # Clear path
    ctrl.Rule(distance['far'], 
             (right_motor['slow'], left_motor['slow'])),
    
    # Medium distance adjustments
    ctrl.Rule(distance['med'] & yaw_angle['port'], 
             (right_motor['stop'], left_motor['fast'])),
    ctrl.Rule(distance['med'] & yaw_angle['port'], 
             (right_motor['fast'], left_motor['stop']))
]   

# Create control system
fis = ctrl.ControlSystem(rules)
fis_sim = ctrl.ControlSystemSimulation(fis)

print("Starting fuzzy inference system")
try:
    while True:
        cycle_start = time.time()
        
        # Read new data
        distances = read_data_zoh()
        
        # Update only if valid data received
        if distances is not None:
            dr, dc, dl = distances
            min_dist = min(dr, dc, dl)
            
            # Update ZOH storage
            last_valid_data['distance'] = min(min_dist, 2000)
            last_valid_data['yaw'] = -30 if min_dist == dr else 0 if min_dist == dc else 30

        # Use ZOH values for FIS
        fis_sim.input['Distance'] = last_valid_data['distance']
        fis_sim.input['Yaw_angle'] = last_valid_data['yaw']
        
        print(f"Distance: {last_valid_data['distance']}, Yaw: {last_valid_data['yaw']}")
        try:
            fis_sim.compute()
        except Exception as e:
            print(f"FIS error: {e}")
            # Fallback to safe values
            right_pwm = MIN_PWM
            left_pwm = MIN_PWM
        else:
            right_pwm = fis_sim.output.get('Right_Motor_Speed', MIN_PWM)
            left_pwm = fis_sim.output.get('Left_Motor_Speed', MIN_PWM)
        

        set_throttle(R_PWM_PIN, right_pwm)
        set_throttle(L_PWM_PIN, left_pwm)
        print(f"Computed PWM - R: {right_pwm}, L: {left_pwm}")
        
        # Precision timing
        elapsed = time.time() - cycle_start
        sleep_time = TS - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print(f"Cycle overrun: {-sleep_time:.3f}s")

except KeyboardInterrupt:
    print("Stopping...")
    set_throttle(R_PWM_PIN, MIN_PWM)
    set_throttle(L_PWM_PIN, MIN_PWM)
    pi.stop()