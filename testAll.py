import time
import board
import digitalio
import adafruit_vl53l1x
from motor_control import Motor

# -----------------------------
# User settings
# -----------------------------
OBSTACLE_THRESHOLD_MM = 200   # obstacle if <= 200 mm
DISTANCE_MODE = 1             # 1 = SHORT, 2 = LONG
TIMING_BUDGET_MS = 100        # sensor timing budget
LOOP_DELAY_S = 1

# -----------------------------
# Shared I2C
# -----------------------------
i2c = board.I2C()

# -----------------------------
# XSHUT pins: Left, Center, Right
# -----------------------------
xshut_pins = [
    digitalio.DigitalInOut(board.D17),  # Left
    digitalio.DigitalInOut(board.D27),  # Center
    digitalio.DigitalInOut(board.D22),  # Right
]

# New addresses for the 3 sensors
new_addresses = [0x30, 0x31, 0x32]

# Turn all sensors OFF first
for pin in xshut_pins:
    pin.switch_to_output(value=False)

time.sleep(0.1)

sensors = []

# Bring sensors up one by one and assign addresses
for pin, addr in zip(xshut_pins, new_addresses):
    pin.value = True
    time.sleep(0.1)

    sensor = adafruit_vl53l1x.VL53L1X(i2c)
    sensor.set_address(addr)
    sensor.distance_mode = DISTANCE_MODE
    sensor.timing_budget = TIMING_BUDGET_MS
    sensors.append(sensor)

# Optional bus scan
if i2c.try_lock():
    try:
        print("I2C addresses found:", [hex(a) for a in i2c.scan()])
    finally:
        i2c.unlock()

# Start ranging
for sensor in sensors:
    sensor.start_ranging()

def cm_to_mm(dist_cm):
    if dist_cm is None:
        return None
    return int(dist_cm * 10)

def obstacle_label(dist_mm, threshold_mm):
    if dist_mm is None:
        return "NO DATA"
    return "BLOCKED" if dist_mm <= threshold_mm else "CLEAR"

print("Reading Left / Center / Right VL53L1X sensors in mm. Ctrl+C to stop.")

motor = Motor()

try:
    while True:
        names = ["LEFT", "CENTER", "RIGHT"]
        distances_mm = [None, None, None]

        for i, sensor in enumerate(sensors):
            if sensor.data_ready:
                # Adafruit library returns distance in cm
                dist_cm = sensor.distance
                distances_mm[i] = cm_to_mm(dist_cm)
                sensor.clear_interrupt()

        left_mm, center_mm, right_mm = distances_mm

        left_state = obstacle_label(left_mm, OBSTACLE_THRESHOLD_MM)
        center_state = obstacle_label(center_mm, OBSTACLE_THRESHOLD_MM)
        right_state = obstacle_label(right_mm, OBSTACLE_THRESHOLD_MM)

        print(
            f"L: {left_mm if left_mm is not None else '---'} mm [{left_state}] | "
            f"C: {center_mm if center_mm is not None else '---'} mm [{center_state}] | "
            f"R: {right_mm if right_mm is not None else '---'} mm [{right_state}]"
        )

        # Simple navigation hint
        if center_mm is not None and center_mm <= OBSTACLE_THRESHOLD_MM:
            if left_mm is not None and right_mm is not None and left_mm <= OBSTACLE_THRESHOLD_MM and right_mm <= OBSTACLE_THRESHOLD_MM:
                decision = "STOP"
                motor.stop()
            elif right_mm is None:
                decision = "TURN LEFT"
                motor.move_left(100, 100)
            elif left_mm is None:
                decision = "TURN RIGHT"
                motor.move_right(100, 100)
            elif left_mm > right_mm:
                decision = "TURN LEFT"
                motor.move_left(100, 100)
            else:
                decision = "TURN RIGHT"
                motor.move_right(100, 100)
        elif left_mm is not None and left_mm <= OBSTACLE_THRESHOLD_MM:
            decision = "TURN RIGHT"
            motor.move_right(100, 100)

        elif right_mm is not None and right_mm <= OBSTACLE_THRESHOLD_MM:
            decision = "TURN LEFT"
            motor.move_left(100, 100)
        else:
            decision = "FORWARD"
            motor.move_forward(100, 100)

        print("Decision:", decision)
        print("-" * 70)

        time.sleep(LOOP_DELAY_S)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    for sensor in sensors:
        try:
            sensor.stop_ranging()
        except Exception:
            pass

    for pin in xshut_pins:
        pin.deinit()
