import vn100
import time

imu = vn100.vn100()
imu.load_config("config.cfg")

if not imu.enabled():
    print("IMU is disabled in config.")
    exit(1)

try:
    imu.open()
    print("IMU opened successfully. Reading data...")

    packet_count = 0
    start_time = time.time()
    last_ypr = None

    while True:
        has_data, ypr_deg, ang_rate, quat, accel = imu.get_latest_data()

        if has_data:
            if ypr_deg != last_ypr:
                print("Original YPR (deg):", ypr_deg)
                print("Angular Rate:", ang_rate)
                print("Quaternion:", quat)
                print("Acceleration:", accel)

                last_ypr = ypr_deg
                packet_count += 1

                current_time = time.time()
                elapsed = current_time - start_time

                if elapsed >= 0.5:
                    frequency = packet_count / elapsed
                    print("Data flow frequency:", frequency, "Hz")
                    packet_count = 0
                    start_time = current_time

        time.sleep(0.001)

except Exception as e:
    print("Error:", e)
finally:
    imu.close()