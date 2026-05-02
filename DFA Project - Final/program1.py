import time
import json
import psutil
import paho.mqtt.client as mqtt

# CONFIGURATION
BROKER = "localhost"
TOPIC = "sensors/cpu"

# SETUP THE CONNECTION
client = mqtt.Client()

print("Connecting to Broker...")
try:
    client.connect(BROKER, 1883, 60)
    print("SUCCESS: Connected to Mosquitto!")
    print(f"Publishing to topic: '{TOPIC}' every 0.01 seconds...")
    print("Press CTRL+C in the terminal to stop.")
except Exception as e:
    print(f"ERROR: Could not connect. Is Mosquitto running? Error: {e}")
    exit()

# START THE BACKGROUND NETWORK LOOP
client.loop_start()

# MAIN LOOP
try:
    while True:
        # 1. Get the data
        cpu_val = psutil.cpu_percent()
        
        # Try to get temperature (Raspberry Pi specific)
        try:
            temp_val = psutil.sensors_temperatures()['cpu_thermal'][0].current
        except:
            temp_val = 0.0 # Fallback for non-Pi computers
        
        timestamp = time.time()
        
        # 2. Package the data (JSON format)
        payload = {
            "timestamp": timestamp,
            "cpu_percent": cpu_val,
            "cpu_temp": temp_val
        }
        
        # 3. Publish (Send) the message
        client.publish(TOPIC, json.dumps(payload))
        
        # 4. Wait 10ms (0.01 seconds)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping Program 1...")
    client.loop_stop()
    client.disconnect()