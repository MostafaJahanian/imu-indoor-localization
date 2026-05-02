import json
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
BROKER = "localhost"
TOPIC = "sensors/cpu"
WINDOW_SIZE = 50  # Large window --> smooth reaction

history_data = []

# --- CALLBACK FUNCTION ---
def on_message(client, userdata, message):
    try:
        # 1. Decode the message
        text_data = message.payload.decode("utf-8")
        data = json.loads(text_data)
        
        # 2. Extract the CPU value
        cpu_val = data.get("cpu_percent")
        temp_val = data.get("cpu_temp")
        
        # 3. Add to history
        if cpu_val is not None and temp_val is not None:
            history_data.append(cpu_val)
            
            # 4. Maintain the sliding window
            if len(history_data) > WINDOW_SIZE:
                history_data.pop(0) # Remove the oldest reading
            
            # 5. Calculate Average
            avg_value = sum(history_data) / len(history_data)
            
            # 6. --- EXPERT SYSTEM RULES (THE "REASONING" PART) ---
            status = "OK"
            
            # RULE 1: Overload Detection
            if avg_value > 90.0:
                status = "CRITICAL: CPU OVERLOAD!"
            
            elif avg_value < 5.0:
                status = "CRITICAL: SUSPICIOUS CPU LOAD!"
            
            # RULE 2: Overheat Detection
            if temp_val > 75.0:
                status = "WARNING: HIGH TEMP!"
                
            # 7. Print a message
            print(f"SLOW : WINDOW SIZE: [{WINDOW_SIZE}] Load: {avg_value:.1f}% | Temp: {temp_val:.1f}°C | Status: {status}")
            
    except Exception as e:
        print(f"Error: {e}")

# --- SETUP SUBSCRIBER ---
client = mqtt.Client()

print("Starting Program 3 (SLOW)...")
try:
    client.connect(BROKER, 1883, 60)
    client.on_message = on_message
    client.subscribe(TOPIC)
    client.loop_forever()
except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Connection Error: {e}")