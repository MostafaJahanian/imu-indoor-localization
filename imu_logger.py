import time
import csv
from sense_hat import SenseHat

# Initialize Sense HAT
sense = SenseHat()
sense.set_imu_config(True, True, True)  # Enable gyro, accel, mag

filename = 'sensor_data_joystick.csv'

print('Logging started...')
print('1. Click UP/DOWN/LEFT/RIGHT to record a STEP.')
print('2. Click MIDDLE to STOP and save.')

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp", "accel_x", "accel_y", "accel_z",
                     "gyro_x", "gyro_y", "gyro_z",
                     "mag_x", "mag_y", "mag_z",
                     "manual_step"])

    try:
        start_time = time.time()
        running = True

        while running:
            # 1. Get Sensor Data
            gyro = sense.get_gyroscope_raw()
            accel = sense.get_accelerometer_raw()
            mag = sense.get_compass_raw()
            now = time.time() - start_time

            # 2. Check Joystick
            step_marker = 0

            # get_events() fetches all clicks since last loop
            events = sense.stick.get_events()
            for event in events:
                if event.action == "pressed":

                    if event.direction == "middle":
                        print("\nMiddle button pressed. Saving and exiting...")
                        running = False  # This breaks the outer loop
                    else:
                        # Any other direction (up, down, left, right) is a step
                        step_marker = 1
                        print(f"Step Recorded! ({event.direction})")

            if not running:
                break  # Exit immediately, don't write the 'stop' click as data

            # 3. Write Data
            writer.writerow([
                round(now, 4),
                round(accel['x'], 4), round(accel['y'], 4), round(accel['z'], 4),
                round(gyro['x'], 4), round(gyro['y'], 4), round(gyro['z'], 4),
                round(mag['x'], 4), round(mag['y'], 4), round(mag['z'], 4),
                step_marker
            ])

            # Flush ensures data is written even if script crashes
            file.flush()

            # Sampling rate ~100Hz
            time.sleep(0.01)

    except KeyboardInterrupt:
        print('\nForce stopped by Ctrl+C. File saved.')

print(f"Data saved to {filename}")
