import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# ---settings---
dt = 1.0                # seconds between GPS readings
num_steps = 50          # how many readings we take
velocity = 2.0          # drone speed in meters per second
gps_noise_std = 3.0     # GPS is typically off by about 3 meters
wind_std = 0.5          # wind randomly pushes the drone about 0.5 meters each second
gps_variance = gps_noise_std ** 2           # variance of GPS noise
wind_variance = wind_std ** 2               # variance of wind noise

# --simulate--
true_positions = []
gps_readings = []

position = 0.0
for step in range(num_steps):
    position = position + velocity * dt + np.random.normal(0, wind_std)
    true_positions.append(position)

    noisy_reading = position + np.random.normal(0, gps_noise_std)
    gps_readings.append(noisy_reading)

# --Predict only tracker--
predicted_positions = []
estimate =0.0

for step in range(num_steps):
    estimate = estimate + velocity * dt
    predicted_positions.append(estimate)

# --Kalman filter prediction--
kalman_estimates = []
estimate = 0.0
uncertainty = 0.0

for gps in gps_readings:
    # Predict
    estimate = estimate + velocity * dt
    uncertainty = uncertainty + wind_variance

    # Update
    K = uncertainty / (uncertainty + gps_variance)
    estimate = estimate + K * (gps - estimate)
    uncertainty = (1 - K) * uncertainty

    kalman_estimates.append(estimate)

# --Measure accuracy--
true_arr = np.array(true_positions)
gps_arr = np.array(gps_readings)
pred_arr = np.array(predicted_positions)
kalman_arr = np.array(kalman_estimates)

gps_rmse = np.sqrt(np.mean((gps_arr - true_arr) ** 2))
pred_rmse = np.sqrt(np.mean((pred_arr - true_arr) ** 2))
kalman_rmse = np.sqrt(np.mean((kalman_arr - true_arr) ** 2))

print(f"Raw GPS error:         {gps_rmse:.2f} m")
print(f"Prediction-only error: {pred_rmse:.2f} m")
print(f"Kalman filter error:   {kalman_rmse:.2f} m")

improvement = (1 - kalman_rmse / gps_rmse) * 100
print(f"Kalman filter reduced error by {improvement:.0f}% vs raw GPS")

# --plot--
plt.plot(true_positions, label="True Positions", linewidth=2)
plt.scatter(range(num_steps), gps_readings, color="red", s=12, label="GPS Readings")
plt.plot(predicted_positions, label="Prediction only", linestyle="--", color="green")
plt.plot(kalman_estimates, label="Kalman filter", color="purple", linewidth=2)
plt.xlabel("Time step (seconds)")
plt.ylabel("Position (meters)")
plt.title("True drone path vs noisy GPS")
plt.legend()
plt.savefig("results.png", dpi=150, bbox_inches="tight")
plt.show()