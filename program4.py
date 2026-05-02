import json
import random
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
BROKER = "localhost"
TOPIC = "sensors/cpu"
WINDOW_SIZE = 50  
SAMPLE_RATE = 0.33 # Target: Keep approx 1/3 of the data (Bernoulli p=0.33)

history_data = []

def on_message(client, userdata, message):
    # --- BERNOULLI SAMPLING STEP ---
    # Generate a random number between 0.0 and 1.0
    # If it is greater than 0.33, IGNORE this message.
    if random.random() > SAMPLE_RATE:
        return  # Skip this data point!

    # If we survive the filter, process as normal:
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
            
            # 6. Monitoring for Mulfuntioning
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
            print(f"SAMPLED : WINDOW SIZE: [{WINDOW_SIZE}] Load: {avg_value:.1f}% | Temp: {temp_val:.1f}°C | Status: {status}")
        
    except Exception as e:
        print(f"Error: {e}")

client = mqtt.Client()
client.connect(BROKER, 1883, 60)
client.on_message = on_message
client.subscribe(TOPIC)

print(f"Program 4 Listening (Sampling ~{int(SAMPLE_RATE*100)}% of data)...")
client.loop_forever()