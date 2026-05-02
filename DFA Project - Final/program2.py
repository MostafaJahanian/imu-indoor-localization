import time
import json
import paho.mqtt.client as mqtt
from sense_hat import SenseHat

# --- CONFIGURATION ---
BROKER = "localhost"
TOPIC = "sensors/sensehat"

# --- SETUP SENSE HAT ---
try:
    sense = SenseHat()
    sense.set_imu_config(True, True, True)  # Enable Compass, Gyro, Accel
    print("Sense HAT initialized.")
except Exception as e:
    print(f"Error initializing Sense HAT: {e}")
    exit()

# --- SETUP MQTT ---
client = mqtt.Client()

print("Connecting to Broker for Sense HAT...")
try:
    client.connect(BROKER, 1883, 60)
    print(f"SUCCESS: Publishing IMU data to '{TOPIC}' every 0.01s")
except Exception as e:
    print(f"ERROR: Could not connect to Mosquitto: {e}")
    exit()

client.loop_start()

# --- MAIN LOOP ---
try:
    while True:
        # 1. Get Sensor Data
        # Orientation (Yaw, Pitch, Roll)
        orientation = sense.get_orientation_degrees()

        # Raw Accelerometer data (g)
        accel = sense.get_accelerometer_raw()

        # Raw Gyroscope data
        # NOTE: Verify if this is rad/s or deg/s during your "L" test!
        gyro = sense.get_gyroscope_raw()

        timestamp = time.time()

        # 2. Format Payload (JSON)
        payload = {
            "timestamp": timestamp,
            # REQUIRED by Assignment: "Predicted Location"
            # We send 0,0 for now since the filter is running offline in Part 2
            "predicted_location": {
                "x": 0.0,
                "y": 0.0
            },
            "orientation": {
                "roll": round(orientation["roll"], 2),
                "pitch": round(orientation["pitch"], 2),
                "yaw": round(orientation["yaw"], 2)
            },
            "accel": {
                "x": round(accel["x"], 4),
                "y": round(accel["y"], 4),
                "z": round(accel["z"], 4)
            },
            "gyro": {
                "x": round(gyro["x"], 4),
                "y": round(gyro["y"], 4),
                "z": round(gyro["z"], 4)
            }
        }

        # 3. Publish
        client.publish(TOPIC, json.dumps(payload))

        # 4. Wait 10ms (approx)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping Program 2...")
    client.loop_stop()
    client.disconnect()