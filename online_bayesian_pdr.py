import time
import json
import csv
import numpy as np
import paho.mqtt.client as mqtt
from sense_hat import SenseHat
from PIL import Image, ImageOps
from scipy.ndimage import gaussian_filter, shift


# ==========================================
# PART 1: BAYESIAN GRID FILTER CLASS
# ==========================================
class BayesianGridFilter:
    def __init__(self, map_path, real_width_meters, start_x, start_y, cell_size=0.20):
        print(f"[BAYES] Initializing Filter (Cell Size: {cell_size}m)...")
        self.cell_size = cell_size

        # 1. LOAD AND PROCESS MAP
        self.occ, self.ppm, self.offset = self._load_map(map_path, real_width_meters)
        self.H, self.W = self.occ.shape
        print(f"[BAYES] Grid Size: {self.W} x {self.H}")

        # 2. PRE-COMPUTE HEADING PDF (The "Hallway Snapping" logic)
        print("[BAYES] Pre-computing Hallway Flow (This takes ~15 seconds)...")
        self.heading_pdf = self._compute_heading_pdf()
        print("[BAYES] Pre-computation Complete.")

        # 3. INITIALIZE BELIEF (Probability Grid)
        self.belief = np.zeros((self.H, self.W))

        # Start with a Gaussian blob around the start position
        sy, sx = self._meters_to_grid(start_x, start_y)
        self.belief[sy, sx] = 1.0
        self.belief = gaussian_filter(self.belief, sigma=2.0)  # Blur it a bit
        self._normalize()

    def _load_map(self, path, real_width):
        """Replaces friend's map_loader.py using PIL"""
        img = Image.open(path).convert('L')
        w_px, h_px = img.size

        # Scale to match target cell size
        pixels_per_meter = w_px / real_width
        scale_factor = (1.0 / self.cell_size) / pixels_per_meter
        new_w = int(w_px * scale_factor)
        new_h = int(h_px * scale_factor)

        img_resized = img.resize((new_w, new_h), Image.Resampling.NEAREST)

        # 0=Wall (Black), 1=Free (White)
        # Note: Friend's code used 1=Wall, we use boolean convention here for ease
        occ_map = np.array(img_resized)
        is_free = occ_map > 127  # True if walkable

        return is_free, (new_w / real_width), (0, 0)

    def _meters_to_grid(self, x, y):
        # Convert meters to grid index (Flip Y for image coords)
        gx = int(x / self.cell_size)
        gy = int(self.H - (y / self.cell_size))
        return max(0, min(self.H - 1, gy)), max(0, min(self.W - 1, gx))

    def _grid_to_meters(self, gy, gx):
        # Convert grid index back to meters
        x = gx * self.cell_size
        y = (self.H - gy) * self.cell_size
        return x, y

    def _compute_heading_pdf(self):
        """
        Simplified Raycasting from friend's heading_pdf_from_map.py
        Calculates 'how well does this cell align with a hallway?'
        """
        pdf = np.ones((self.H, self.W)) * 0.1  # Baseline probability

        # Directions to check (Up, Down, Left, Right)
        # We simplify to 4 cardinal directions for speed on Pi
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for y in range(self.H):
            for x in range(self.W):
                if not self.occ[y, x]: continue  # Skip walls

                # Check how much open space is around
                free_space = 0
                for dy, dx in directions:
                    length = 0
                    for r in range(1, 10):  # Raycast up to 2 meters
                        ny, nx = y + dy * r, x + dx * r
                        if 0 <= ny < self.H and 0 <= nx < self.W and self.occ[ny, nx]:
                            length += 1
                        else:
                            break
                    free_space += length

                # Cells with long straight paths get higher weight
                pdf[y, x] += free_space * 0.1

        return pdf

    def predict(self, stride, heading_rad):
        """
        Shift the probability grid based on movement.
        """
        # Calculate shift in grid cells
        dx = stride * np.cos(heading_rad)
        dy = stride * np.sin(heading_rad)

        shift_x = dx / self.cell_size
        shift_y = -(dy / self.cell_size)  # Negative because Y is index (top-down)

        # 1. SHIFT (Move the probability blob)
        # We use scipy.ndimage.shift with spline interpolation
        self.belief = shift(self.belief, shift=[shift_y, shift_x], order=1, mode='constant', cval=0.0)

        # 2. DIFFUSE (Add noise/uncertainty)
        # We blur the grid slightly to account for stride error
        self.belief = gaussian_filter(self.belief, sigma=0.8)

        # 3. MASK (Remove probability that moved into walls)
        self.belief[~self.occ] = 0.0
        self._normalize()

    def update(self):
        """
        Bayesian Update: Multiply by Map Likelihood (Heading PDF)
        """
        # In the friend's code, this "snaps" the position to valid hallways
        self.belief *= self.heading_pdf
        self._normalize()

    def estimate(self):
        """
        Return the coordinates of the highest probability cell (Mode)
        """
        # Find index of max value
        max_idx = np.unravel_index(np.argmax(self.belief), self.belief.shape)
        gy, gx = max_idx
        return self._grid_to_meters(gy, gx) + (0.0,)  # Return (x, y, 0) to match PF format

    def _normalize(self):
        s = np.sum(self.belief)
        if s > 1e-9:
            self.belief /= s
        else:
            # If we get lost (sum=0), re-initialize uniformly on free space
            self.belief[self.occ] = 1.0
            self.belief /= np.sum(self.belief)


## ... (Previous Class definitions remain the same)

# ==========================================
# PART 2: MAIN EXECUTION
# ==========================================
# --- LOAD CONFIGURATION ---
try:
    with open('config.json', 'r') as f:
        config = json.load(f)
        print("[INIT] Configuration loaded from config.json")
except FileNotFoundError:
    print("[ERROR] config.json not found!")
    exit()

BROKER = config['mqtt_broker']
TOPIC = config['mqtt_topic_sensehat']

MAP_FILE = config['map_settings']['filename']
REAL_WIDTH = config['map_settings']['real_width_meters']
CELL_SIZE = config['map_settings']['cell_size']

step_length = config['pdr_settings']['step_length']
START_X = config['pdr_settings']['start_x']
START_Y = config['pdr_settings']['start_y']
START_H = config['pdr_settings']['start_heading']

# --- HARDWARE INIT ---
sense = SenseHat()
sense.set_imu_config(True, True, True)

client = mqtt.Client()
try:
    client.connect(BROKER, 1883, 60)
    client.loop_start()
except:
    pass

# --- INIT BAYESIAN FILTER ---
# Note: We now use CELL_SIZE from the config
bf = BayesianGridFilter(MAP_FILE, REAL_WIDTH, START_X, START_Y, cell_size=CELL_SIZE)


# Gyro Variables
gyro_bias = 0.0
# We track absolute heading by integrating gyro ourselves
current_heading = START_H  # Start facing West

print("\n" + "=" * 40)
print("  BAYESIAN PDR READY")
print("  Walk -> Click. Middle -> Stop.")
print("=" * 40 + "\n")

with open('bayesian_run_log.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['ts', 'x', 'y', 'theta', 'step'])

    last_time = time.time()
    running = True

    # Init calibration window
    bias_window = []

    try:
        while running:
            now = time.time()
            dt = now - last_time
            last_time = now

            # 1. SENSOR
            raw_gz = sense.get_gyroscope_raw()['z']

            # Quick Calibration (First 2 seconds)
            if len(bias_window) < 200:
                bias_window.append(raw_gz)
                gyro_bias = sum(bias_window) / len(bias_window)
                continue  # Skip logic until calibrated

            # Continuous Heading Integration
            # We must integrate continuously, not just on steps, for the Bayesian Grid to know direction
            d_theta = -(raw_gz - gyro_bias) * dt
            current_heading += d_theta
            current_heading = (current_heading + np.pi) % (2 * np.pi) - np.pi  # Wrap -pi to pi

            # 2. JOYSTICK
            is_step = False
            for event in sense.stick.get_events():
                if event.action == "pressed":
                    if event.direction == "middle":
                        running = False
                        print("[STOP] Saving...")
                    else:
                        is_step = True
                        print(f"[STEP] Heading: {np.degrees(current_heading):.0f}")

            if not running: break

            # 3. FILTER LOGIC (On Step Only)
            if is_step:
                # PREDICT: Shift the probability grid
                bf.predict(step_length, current_heading)

                # UPDATE: Apply map constraints
                bf.update()

                # ESTIMATE
                est_x, est_y, _ = bf.estimate()

                # OUTPUT
                payload = {
                    "timestamp": now,
                    "predicted_location": {"x": round(est_x, 2), "y": round(est_y, 2)},
                    "heading": round(current_heading, 2),
                    "step_detected": True
                }
                client.publish(TOPIC, json.dumps(payload))
                writer.writerow([now, est_x, est_y, current_heading, 1])
                f.flush()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        client.loop_stop()
        print("[SAVED] Log saved to bayesian_run_log.csv")