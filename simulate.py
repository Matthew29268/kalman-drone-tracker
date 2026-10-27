import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# ---settings---
dt = 1.0                # seconds between GPS readings
num_steps = 50          # how many readings we take
velocity = 2.0          # drone speed in meters per second
gps_noise_std = 3.0     # GPS is typically off by about 3 meters

# --simulate--
true_positions = []
gps_readings = []

position = 0.0
for step in range(num_steps):
    position = position + velocity * dt
    true_positions.append(position)

    noisy_reading = position + np.random.normal(0, gps_noise_std)
    gps_readings.append(noisy_reading)

# --plot--
plt.plot(true_positions, label="True Positions", linewidth=2)
plt.scatter(range(num_steps), gps_readings, color="red", s=12, label="GPS Readings")
plt.xlabel("Time step (seconds)")
plt.ylabel("Position (meters)")
plt.title("True drone path vs noisy GPS")
plt.legend()
plt.show()