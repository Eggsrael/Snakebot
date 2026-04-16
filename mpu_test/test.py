import mpu6050
import time

# -----------------------------
# Initialize MPU-6050
# -----------------------------
mpu = mpu6050.mpu6050(0x68)  # default address

# set gyro range for more/less sensitivity
mpu.set_gyro_range(mpu.GYRO_RANGE_500DEG)  # 250, 500, 1000, 2000

# -----------------------------
# Parameters
# -----------------------------
GYRO_THRESHOLD_DEG = 2.0  # or 2.0 ,degrees/sec threshold for veering
SAMPLE_TIME = 0.5         # seconds between readings
#offset = 1.30

# -----------------------------
# Main loop
# -----------------------------
try:
    print("Monitoring robot straightness (Ctrl+C to stop)...")
    while True:
        gyro_data = mpu.get_gyro_data()  # returns dict with x, y, z
        x_rate_deg = gyro_data['x']      # X-axis
        #z_rate_deg += offset

        if abs(x_rate_deg) < GYRO_THRESHOLD_DEG:
            print(f"Moving straight | X-gyro: {x_rate_deg:.2f} deg/sec")
        else:
            direction = "left" if x_rate_deg > 0 else "right"
            print(f"Veering {direction} | X-gyro: {x_rate_deg:.2f} deg/sec")

        time.sleep(SAMPLE_TIME)

except KeyboardInterrupt:
    print("Stopped monitoring.")
