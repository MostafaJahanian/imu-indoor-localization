# IMU-Based Indoor Localization — Particle Filter & Bayesian Grid Filter

A Pedestrian Dead Reckoning (PDR) system for GPS-denied indoor environments, built on a Raspberry Pi 4 with a Sense HAT IMU. The system fuses gyroscope-based heading estimation with a 2D floor plan to track a user's position in real time using two probabilistic approaches: a Particle Filter and a Bayesian Grid Filter.

> Developed as a group project for the Data Fusion Architecture (DFA) course, MSc Smart Systems Engineering, Hanze University of Applied Sciences (January 2026).

---

## Team

Developed collaboratively by a 3-person team as part of the MSc Smart Systems Engineering programme at Hanze University.

My contributions focused on the Particle Filter implementation and the DSMS monitoring stack.

---

## System Overview

```
Raspberry Pi (Sense HAT)
        │
        ├── imu_logger.py         # Raw IMU + joystick data logger
        ├── program2.py           # Live IMU publisher via MQTT
        │
        └── MQTT Broker (localhost)
                │
                ├── online_particle_pdr.py    # Particle Filter navigator
                ├── online_bayesian_pdr.py    # Bayesian Grid navigator
                │
                └── DSMS Stack
                        ├── program1.py         # CPU load publisher
                        ├── program3_fast.py    # Fast sliding window monitor
                        ├── program3_slow.py    # Slow sliding window monitor
                        └── program4.py         # Bernoulli sampler
```

---

## Key Features

- **Gyroscope bias calibration:**  5-second stationary window at startup removes Zero-Rate Output (ZRO) drift
- **Joystick-triggered step detection:**  Eliminates automated step detection errors; user clicks to register each step
- **Two localization methods:**  Running on the same hardware for direct comparison
- **Map-constrained filtering:**  Floor plan used as binary occupancy grid to reject physically impossible trajectories
- **Kidnapped robot recovery:**  Particle Filter automatically resets if all particles hit walls simultaneously
- **Real-time MQTT communication:**  Decoupled publisher/subscriber architecture between Pi and laptop
- **DSMS monitoring stack:**  Lightweight system health monitor running in parallel with navigation

---

## File Descriptions

### Navigation & Sensing

| File | Role |
|---|---|
| `imu_logger.py` | Runs on the Raspberry Pi. Reads gyroscope, accelerometer, and magnetometer data from the Sense HAT at ~100Hz and logs it to a CSV. Joystick direction keys register a step event; middle button stops logging. Used for offline data collection. |
| `program2.py` | Runs on the Raspberry Pi. Publishes live IMU data (orientation, accelerometer, gyroscope) to the MQTT broker on the `sensors/sensehat` topic at ~100Hz. This is the real-time data source for the online navigation scripts. |
| `online_particle_pdr.py` | Runs on the Raspberry Pi. Subscribes to the MQTT IMU stream and implements a Sequential Monte Carlo (Particle Filter) for real-time localization. Maintains 500 weighted particles, each with its own (x, y, θ) state. Applies a stop-on-wall collision model, multi-factor weight updates (map validity, stride consistency, heading consistency), and systematic resampling when effective sample size drops below N/2. Publishes position estimates back to MQTT and logs them to `data/particle_run_log.csv`. |
| `online_bayesian_pdr.py` | Runs on the Raspberry Pi. Subscribes to the MQTT IMU stream and implements a Discrete Grid-Based Bayesian Filter for real-time localization. Maintains a full probability distribution over a 2D grid of the floor plan. Applies prediction via grid shifting and Gaussian diffusion, and correction via map-based likelihood masking. Publishes position estimates to MQTT and logs them to `data/bayesian_run_log.csv`. |
| `config.json` | Central configuration file. Defines MQTT broker address, topics, map settings (filename, real-world width, cell size), PDR settings (step length, start position and heading), and particle filter parameters. Both online scripts read from this file at startup. |

### DSMS Monitoring Stack

| File | Role |
|---|---|
| `program1.py` | Runs on the Raspberry Pi. Publishes CPU load (%) and temperature (°C) to the `sensors/cpu` MQTT topic every 10ms using `psutil`. Acts as the synthetic load generator for the DSMS pipeline. |
| `program3_fast.py` | Runs on the Raspberry Pi. Subscribes to `sensors/cpu` and computes a sliding window average with window size 5. Provides high-resolution, low-latency feedback sensitive to short spikes. Includes rule-based alerts for overload (>90%) and overheating (>75°C). |
| `program3_slow.py` | Same as above but with window size 50. Acts as a low-pass filter, smoothing out momentary spikes to reveal long-term CPU load trends. |
| `program4.py` | Runs on the Raspberry Pi. Subscribes to `sensors/cpu` and applies Bernoulli sampling, keeping only ~33% of incoming packets. Demonstrates that statistical properties (mean CPU load) can still be accurately estimated with significantly reduced data transmission — validating the DSMS bandwidth optimization hypothesis. |

---

## Results

| Method | Map Consistency | Segment Wall Crossings | Endpoint Corridor Score |
|---|---|---|---|
| Bayesian Grid Filter | 95.2% | 1 | 116 px (mid-corridor) |
| Particle Filter | 90.5% | 2 | 35 px (near wall) |

- Both filters ran in real time on Raspberry Pi 4 at ~18% total CPU load
- Particle Filter: computationally efficient, sensitive to heading drift
- Bayesian Grid Filter: stronger global map consistency, higher computational cost

---

## Project Structure

```
imu-indoor-localization/
│
├── DFA_report-final.ipynb      # Full analysis notebook
├── DFA_report-final.pdf        # Academic report
├── map.png                     # Floor plan (binary occupancy map)
├── config.json                 # System configuration
│
├── online_particle_pdr.py      # Particle Filter (real-time)
├── online_bayesian_pdr.py      # Bayesian Grid Filter (real-time)
├── imu_logger.py               # IMU data logger (runs on Pi)
├── program1.py                 # DSMS: CPU load publisher
├── program2.py                 # DSMS: IMU data publisher
├── program3_fast.py            # DSMS: Fast sliding window monitor
├── program3_slow.py            # DSMS: Slow sliding window monitor
├── program4.py                 # DSMS: Bernoulli sampler
│
├── data/
│   ├── particle_run_log.csv
│   ├── bayesian_run_log.csv
│   ├── particle_session_fast.log
│   ├── particle_session_slow.log
│   ├── particle_session_sampled.log
│   ├── bayes_session_fast.log
│   ├── bayes_session_slow.log
│   └── bayes_session_sampled.log
│
├── requirements.txt
├── .gitignore
└── README.md
```

---

## Installation

```bash
git clone https://github.com/MostafaJahanian/imu-indoor-localization.git
cd imu-indoor-localization
pip install -r requirements.txt
```

**On Raspberry Pi additionally:**
```bash
pip install sense-hat
```

---

## Usage

### Step 1 — Start the MQTT broker
```bash
mosquitto
```

### Step 2 — On the Raspberry Pi
```bash
# Publish live IMU data
python program2.py

# Publish CPU monitoring data (separate terminal)
python program1.py

# Run DSMS monitors (separate terminals)
python program3_fast.py
python program3_slow.py
python program4.py

# Run navigation (separate terminal)
python online_particle_pdr.py
# OR
python online_bayesian_pdr.py
```

### Step 3 — On the laptop

Open `DFA_report-final.ipynb` and run all cells. Logged data in `data/` is used to reproduce all visualizations from the report.

---

## Tech Stack

`Python` `NumPy` `SciPy` `Pandas` `Matplotlib` `Paho-MQTT` `Sense HAT` `Raspberry Pi` `Particle Filter` `Bayesian Filter` `MQTT` `asyncio`

---

## Academic Report

Full technical report covering system architecture, mathematical models, experimental results, sensitivity analysis, and hardware performance is available in [`DFA_report-final.pdf`](./DFA_report-final.pdf).

---

## Author

**Mostafa Jahanian**
[LinkedIn](https://linkedin.com/in/mostafa-jahanian) · [GitHub](https://github.com/MostafaJahanian)
