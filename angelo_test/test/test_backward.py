import pigpio
import time

# GPIO pins
R_PWM_PIN = 12
L_PWM_PIN = 13

# ESC endpoints (in microseconds)
NEUTRAL = 1500   # stop / mid-throttle
MAX_FWD = 2000   # full forward
MAX_REV = 1000   # full reverse

# Ramp configuration
# Each “stage” (forward or reverse) lasts 5 seconds total:
#  - 2.5 s to go from NEUTRAL → endpoint
#  - 2.5 s to return endpoint → NEUTRAL
RAMP_DURATION = 2.5      # seconds for each half of a stage
RAMP_STEPS    = 50       # discrete steps in each half of a stage

# Compute per-step increments and delays
FWD_STEP    = (MAX_FWD - NEUTRAL) / RAMP_STEPS
REV_STEP    = (NEUTRAL - MAX_REV) / RAMP_STEPS
STEP_DELAY  = RAMP_DURATION / RAMP_STEPS

def set_throttle(pi, pin, pw):
    """Clamp pw to [MAX_REV..MAX_FWD] and send as a servo pulse."""
    if pw < MAX_REV:
        pw = MAX_REV
    elif pw > MAX_FWD:
        pw = MAX_FWD
    pi.set_servo_pulsewidth(pin, int(pw))

def setup_escs(pi):
    """Arm both ESCs by sending NEUTRAL for 2 seconds."""
    print("▶ Arming ESCs at NEUTRAL (1500 µs) for 2 seconds...")
    for pin in (R_PWM_PIN, L_PWM_PIN):
        set_throttle(pi, pin, NEUTRAL)
    time.sleep(2)
    print("✅ ESCs are now armed.\n")

def cleanup_escs(pi):
    """
    Disarm ESCs cleanly:
      1) Ensure both ESCs sit at NEUTRAL (1500 µs) for a moment
      2) Then remove pulses entirely (0 µs)
    """
    print("\n▶ Disarming ESCs…")
    # 1) Bring both back to NEUTRAL and hold briefly
    try:
        for pin in (R_PWM_PIN, L_PWM_PIN):
            set_throttle(pi, pin, NEUTRAL)
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    # 2) Disable pulses entirely
    try:
        for pin in (R_PWM_PIN, L_PWM_PIN):
            pi.set_servo_pulsewidth(pin, 0)
        time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    # 3) Stop pigpio client
    try:
        pi.stop()
    except Exception:
        pass

    print("✅ ESCs disarmed, exiting.")

def ramp_forward(pi):
    """Ramp from NEUTRAL → MAX_FWD over 2.5 s, then back to NEUTRAL over 2.5 s."""
    print("▶ Ramp FORWARD stage (5 seconds total)…")
    # Ramp up: NEUTRAL → MAX_FWD
    for i in range(1, RAMP_STEPS + 1):
        pwm = NEUTRAL + i * FWD_STEP
        set_throttle(pi, R_PWM_PIN, pwm)
        set_throttle(pi, L_PWM_PIN, pwm)
        time.sleep(STEP_DELAY)

    # Ramp down: MAX_FWD → NEUTRAL
    for i in range(RAMP_STEPS, 0, -1):
        pwm = NEUTRAL + i * FWD_STEP
        set_throttle(pi, R_PWM_PIN, pwm)
        set_throttle(pi, L_PWM_PIN, pwm)
        time.sleep(STEP_DELAY)

    # Ensure exactly NEUTRAL at end
    set_throttle(pi, R_PWM_PIN, NEUTRAL)
    set_throttle(pi, L_PWM_PIN, NEUTRAL)
    print("▶ Forward stage complete.\n")
    # Brief pause before next stage
    time.sleep(0.5)

def ramp_reverse(pi):
    """Ramp from NEUTRAL → MAX_REV over 2.5 s, then back to NEUTRAL over 2.5 s."""
    print("▶ Ramp REVERSE stage (5 seconds total)…")
    # Ramp down: NEUTRAL → MAX_REV
    for i in range(1, RAMP_STEPS + 1):
        pwm = NEUTRAL - i * REV_STEP
        set_throttle(pi, R_PWM_PIN, pwm)
        set_throttle(pi, L_PWM_PIN, pwm)
        time.sleep(STEP_DELAY)

    # Ramp up: MAX_REV → NEUTRAL
    for i in range(RAMP_STEPS, 0, -1):
        pwm = NEUTRAL - i * REV_STEP
        set_throttle(pi, R_PWM_PIN, pwm)
        set_throttle(pi, L_PWM_PIN, pwm)
        time.sleep(STEP_DELAY)

    # Ensure exactly NEUTRAL at end
    set_throttle(pi, R_PWM_PIN, NEUTRAL)
    set_throttle(pi, L_PWM_PIN, NEUTRAL)
    print("▶ Reverse stage complete.\n")
    # Brief pause before cleanup
    time.sleep(0.5)

if __name__ == "__main__":
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: Could not connect to pigpiod. Is the daemon running?")

    try:
        setup_escs(pi)

        # Perform one 5-second forward stage
        ramp_forward(pi)

        # Perform one 5-second reverse stage
        ramp_reverse(pi)

        print("🎉 Calibration/test sequence complete. Both ESCs at NEUTRAL.")

    except KeyboardInterrupt:
        print("\n▶ Interrupted by user.")

    finally:
        cleanup_escs(pi)
