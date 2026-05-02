import time
import json
import csv
import numpy as np
import paho.mqtt.client as mqtt
from sense_hat import SenseHat
from PIL import Image, ImageOps
import collections


# ==========================================
# MODULE 1: MAP PROCESSOR
# ==========================================
def process_map_and_scale(image_path, real_width_meters):
    """
    Loads a map, pads it to be square (black padding), and calculates the
    pixel-to-meter conversion rate.

    Args:
        image_path (str): Path to the map image file.
        real_width_meters (float): The physical width of the ORIGINAL image in meters.

    Returns:
        tuple: (square_map_array, pixels_per_meter, map_offset)
               map_offset is (offset_x_pixels, offset_y_pixels) from padding
    """
    # 1. Load the Image
    # Convert to grayscale ('L') so we deal with 0-255 values
    original_img = Image.open(image_path).convert('L')
    orig_w, orig_h = original_img.size

    print(f"Original Map Size: {orig_w} x {orig_h} pixels")

    # 2. Calculate Padding (Make it Square)
    max_dim = max(orig_w, orig_h)

    # Calculate how much padding to add to width and height
    # We will center the image in the new square
    delta_w = max_dim - orig_w
    delta_h = max_dim - orig_h

    padding = (delta_w // 2, delta_h // 2, delta_w - (delta_w // 2), delta_h - (delta_h // 2))

    # 3. Apply Padding (0 = Black)
    # ImageOps.expand adds borders. We fill with 0 (Black).
    square_img = ImageOps.expand(original_img, padding, fill=0)

    print(f"New Map Size: {square_img.size[0]} x {square_img.size[1]} pixels")

    # 4. Calculate Conversion Rate
    # Scale is based on the ORIGINAL width, because padding is just empty space.
    # pixels_per_meter = Original Pixels / Real Meters
    pixels_per_meter = orig_w / real_width_meters

    print(f"Scale: {pixels_per_meter:.2f} pixels per meter")

    # 5. Store the padding offset (IMPORTANT for coordinate transformation!)
    map_offset = (delta_w // 2, delta_h // 2)
    print(f"Map Offset: {map_offset} pixels")

    # 6. Convert to Numpy Array (0=Wall, 255=Walkable)
    map_array = np.array(square_img)

    return map_array, pixels_per_meter, map_offset


def world_to_pixel(x_meters, y_meters, pixels_per_meter, map_offset, map_height_pixels):
    """
    Convert from world coordinates (meters, bottom-left origin)
    to pixel coordinates (pixels, top-left origin)

    Args:
        x_meters, y_meters: Position in meters (world frame)
        pixels_per_meter: Scale factor from process_map_and_scale()
        map_offset: (offset_x, offset_y) from padding
        map_height_pixels: Total height of the padded square map
    """
    # Convert meters to pixels (relative to original map)
    x_pixels_original = x_meters * pixels_per_meter
    y_pixels_original = y_meters * pixels_per_meter

    # Add padding offset
    x_pixels = x_pixels_original + map_offset[0]

    # FLIP Y-axis: bottom-left → top-left
    # In image space, y=0 is top, but in world space y=0 is bottom
    y_pixels = map_height_pixels - (y_pixels_original + map_offset[1])

    return int(x_pixels), int(y_pixels)


def pixel_to_world(x_pixels, y_pixels, pixels_per_meter, map_offset, map_height_pixels):
    """
    Convert from pixel coordinates to world coordinates (meters)
    """
    # Remove padding offset
    x_pixels_original = x_pixels - map_offset[0]
    y_pixels_original = map_height_pixels - y_pixels - map_offset[1]  # Flip Y

    # Convert to meters
    x_meters = x_pixels_original / pixels_per_meter
    y_meters = y_pixels_original / pixels_per_meter

    return x_meters, y_meters


def is_walkable(x_meters, y_meters, map_array, pixels_per_meter, map_offset):
    """
    Check if a world-space position is walkable (not a wall)

    Returns:
        bool: True if walkable, False if wall or out of bounds
    """
    map_height_pixels = map_array.shape[0]
    px, py = world_to_pixel(x_meters, y_meters, pixels_per_meter,
                            map_offset, map_height_pixels)

    # Check bounds
    if px < 0 or px >= map_array.shape[1] or py < 0 or py >= map_array.shape[0]:
        return False

    # Check if walkable (255 = white = walkable, 0 = black = wall)
    return map_array[py, px] > 127  # Threshold for white


# ==========================================
# MODULE 2: PARTICLE FILTER
# ==========================================
class SimpleParticleFilter:
    def __init__(self, num_particles, map_array, pixels_per_meter, map_offset,
                 initial_position, position_uncertainty=0.5):
        """
        Simple step-based particle filter.

        Args:
            num_particles: Number of particles
            map_array: Processed map array
            pixels_per_meter: Scale factor
            map_offset: Padding offset
            initial_position: [x, y, theta] in meters and radians
            position_uncertainty: Initial position std (meters)
        """
        self.num_particles = num_particles
        self.map_array = map_array
        self.pixels_per_meter = pixels_per_meter
        self.map_offset = map_offset

        # Map dimensions in meters
        self.map_width_meters = map_array.shape[1] / pixels_per_meter
        self.map_height_meters = map_array.shape[0] / pixels_per_meter

        # Initialize particles around initial position
        self.particles = np.empty((num_particles, 3))
        self.particles[:, 0] = initial_position[0] + np.random.randn(num_particles) * position_uncertainty
        self.particles[:, 1] = initial_position[1] + np.random.randn(num_particles) * position_uncertainty
        self.particles[:, 2] = initial_position[2] + np.random.randn(num_particles) * (np.pi / 8)
        self.particles[:, 2] %= (2 * np.pi)

        # Initialize weights uniformly
        self.weights = np.ones(num_particles) / num_particles

        # For tracking
        self.last_movement = np.zeros(num_particles)

    def predict_simple(self, stride, heading_observed, stride_sigma=0.1, heading_sigma=0.1):
        """
        Predict: Set heading from observation, then move forward.
        Now with wall collision prevention.
        """
        # Set particle headings to observed heading + noise
        noise_heading = np.random.randn(self.num_particles) * heading_sigma
        self.particles[:, 2] = heading_observed + noise_heading
        self.particles[:, 2] %= (2 * np.pi)

        # Move particles forward with noisy stride
        noise_stride = np.random.randn(self.num_particles) * stride_sigma
        dist = stride + noise_stride

        # Calculate proposed new positions
        new_x = self.particles[:, 0] + dist * np.cos(self.particles[:, 2])
        new_y = self.particles[:, 1] + dist * np.sin(self.particles[:, 2])

        # Check each particle for wall collision
        for i in range(self.num_particles):
            # Check if new position is valid
            if (0 <= new_x[i] <= self.map_width_meters and
                    0 <= new_y[i] <= self.map_height_meters and
                    is_walkable(new_x[i], new_y[i], self.map_array,
                                self.pixels_per_meter, self.map_offset)):
                # Valid move - update position
                self.particles[i, 0] = new_x[i]
                self.particles[i, 1] = new_y[i]
                self.last_movement[i] = dist[i]
            else:
                # Hit a wall - DON'T MOVE, stay at current position
                self.last_movement[i] = 0.0

    def update_simple(self, stride_expected, heading_expected,
                      stride_sigma=0.1, heading_sigma=0.1):
        """
        Update: Weight particles by map validity, stride, and heading.

        Args:
            stride_expected: Expected stride length
            heading_expected: Expected heading
            stride_sigma: Stride likelihood std
            heading_sigma: Heading likelihood std
        """
        # 1. Map validity (binary)
        for i in range(self.num_particles):
            x, y = self.particles[i, 0], self.particles[i, 1]

            # Check bounds
            if x < 0 or x > self.map_width_meters or y < 0 or y > self.map_height_meters:
                self.weights[i] = 0.0
                continue

            # Check walkability
            if not is_walkable(x, y, self.map_array, self.pixels_per_meter, self.map_offset):
                self.weights[i] = 0.0

        # 2. Stride likelihood (Gaussian)
        w_stride = np.exp(-((self.last_movement - stride_expected) ** 2) / (2 * stride_sigma ** 2))

        # 3. Heading likelihood (Gaussian)
        heading_diff = self.particles[:, 2] - heading_expected
        # Handle angle wrapping
        heading_diff = (heading_diff + np.pi) % (2 * np.pi) - np.pi
        w_heading = np.exp(-(heading_diff ** 2) / (2 * heading_sigma ** 2))

        # Combine weights
        self.weights *= w_stride * w_heading

        # Normalize
        self.weights += 1e-12  # Avoid division by zero
        weight_sum = np.sum(self.weights)

        if weight_sum < 1e-12:
            print("WARNING: All particles died!")
            self.weights = np.ones(self.num_particles) / self.num_particles
        else:
            self.weights /= weight_sum

    def resample(self):
        """Systematic resampling."""
        if self.neff() < self.num_particles / 2:
            indices = self._systematic_resample()
            self.particles = self.particles[indices]
            self.weights = np.ones(self.num_particles) / self.num_particles

    def _systematic_resample(self):
        """Systematic resampling algorithm."""
        N = self.num_particles
        positions = (np.random.random() + np.arange(N)) / N

        indexes = np.zeros(N, dtype=int)
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0

        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        return indexes

    def neff(self):
        """Calculate effective number of particles."""
        return 1.0 / np.sum(np.square(self.weights))

    def estimate_simple(self):
        """
        Estimate state as weighted average.

        Returns:
            np.array: [x, y, theta]
        """
        x = np.sum(self.particles[:, 0] * self.weights)
        y = np.sum(self.particles[:, 1] * self.weights)

        # Circular mean for heading
        sin_sum = np.sum(np.sin(self.particles[:, 2]) * self.weights)
        cos_sum = np.sum(np.cos(self.particles[:, 2]) * self.weights)
        theta = np.arctan2(sin_sum, cos_sum)

        return np.array([x, y, theta])


def compute_heading_at_step(df, step_idx):
    """
    Compute cumulative heading from start to this step by integrating gyro.
    """
    # Integrate gyro from beginning to this step
    gyro_slice = df.loc[:step_idx, 'gyro_z']  # Use raw gyro (no Kalman needed)
    timestamps = df.loc[:step_idx, 'timestamp']

    heading = np.pi  # Initial heading (West = 180°)

    for i in range(1, len(gyro_slice)):
        dt = timestamps.iloc[i] - timestamps.iloc[i-1]
        heading -= gyro_slice.iloc[i] * dt

    return heading % (2 * np.pi)

# ==========================================
# MODULE 3: MAIN LOOP (REAL-TIME)
# ==========================================
# --- LOAD CONFIGURATION ---
try:
    with open('config.json', 'r') as f:
        config = json.load(f)
        print("[INIT] Configuration loaded from config.json")
except FileNotFoundError:
    print("[ERROR] config.json not found! Using hardcoded defaults.")
    exit()

# Extract Variables
BROKER = config['mqtt_broker']
TOPIC = config['mqtt_topic_sensehat']

MAP_FILE = config['map_settings']['filename']
REAL_WIDTH = config['map_settings']['real_width_meters']

step_length = config['pdr_settings']['step_length']
START_X = config['pdr_settings']['start_x']
START_Y = config['pdr_settings']['start_y']
START_H = config['pdr_settings']['start_heading']

N_PARTICLES = config['particle_filter']['num_particles']

# --- HARDWARE INIT ---
sense = SenseHat()
sense.set_imu_config(True, True, True)

client = mqtt.Client()
try:
    client.connect(BROKER, 1883, 60)
    client.loop_start()
    print("[MQTT] Connected.")
except:
    print("[MQTT] Failed. Running in Offline Mode.")

# --- MAP & PF INIT ---
map_grid, ppm, offset = process_map_and_scale(MAP_FILE, REAL_WIDTH)
pf = SimpleParticleFilter(N_PARTICLES, map_grid, ppm, offset, [START_X, START_Y, START_H])

# --- GYRO CALIBRATION ---
print("\n" + "=" * 40)
print("  CALIBRATING GYRO...")
print("  Keep device COMPLETELY STILL for 5 sec")
print("=" * 40)

gyro_samples = []
for i in range(500):  # 5 seconds at 100Hz
    raw_gz = sense.get_gyroscope_raw()['z']
    gyro_samples.append(raw_gz)
    time.sleep(0.01)
    if i % 100 == 0:
        print(f"  {i/100:.0f}s...")

gyro_bias = np.mean(gyro_samples)
print(f"\n  Gyro bias: {gyro_bias:.5f} rad/s")
print("  Calibration complete!")

# --- HEADING TRACKING (MODIFIED) ---
current_heading = START_H

print("\n" + "=" * 40)
print("  PDR SYSTEM READY")
print("  1. Joystick UP/DOWN/LEFT/RIGHT = Step")
print("  2. Joystick MIDDLE = STOP & SAVE")
print("=" * 40 + "\n")


with open('particle_run_log.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['ts', 'x', 'y', 'theta', 'step'])

    last_time = time.time()
    running = True

    try:
        while running:
            now = time.time()
            dt = now - last_time
            last_time = now

            # A. SENSOR READ & BIAS REMOVAL
            raw_gz = sense.get_gyroscope_raw()['z']

            clean_gz = raw_gz - gyro_bias

            # B. INTEGRATE HEADING CONTINUOUSLY
            # Negate gyro (same as offline: heading -= gyro * dt)
            current_heading -= clean_gz * dt
            current_heading %= (2 * np.pi)

            # C. JOYSTICK LOGIC
            is_step = False
            for event in sense.stick.get_events():
                if event.action == "pressed":
                    if event.direction == "middle":
                        print(f"\n[STOP] Middle Button Pressed. Saving...")
                        running = False
                    else:
                        is_step = True
                        print(f"[STEP] @ {now:.1f}s | Heading: {np.degrees(current_heading):.0f}°")

            if not running:
                break

            # D. PARTICLE FILTER (ONLY WHEN STEP DETECTED)
            if is_step:
                # Use same logic as offline: predict → update → resample → estimate
                pf.predict_simple(step_length, current_heading)
                pf.update_simple(step_length, current_heading)
                pf.resample()

                # Get estimate
                est = pf.estimate_simple()

                # MQTT Live View
                payload = {
                    "timestamp": now,
                    "predicted_location": {"x": round(est[0], 2), "y": round(est[1], 2)},
                    "heading": round(est[2], 2),
                    "step_detected": True
                }
                client.publish(TOPIC, json.dumps(payload))

                # Log File
                writer.writerow([now, est[0], est[1], est[2], 1])

                print(f"  Position: ({est[0]:.2f}, {est[1]:.2f}) m")

            # Small delay to prevent CPU overload
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[STOP] Force Quit.")
    finally:
        client.loop_stop()
        print("[SAVED] Log saved to particle_run_log.csv")