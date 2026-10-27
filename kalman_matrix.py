import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# --- Settings ---
dt = 1.0
num_steps = 50
true_velocity = 2.0
gps_noise_std = 3.0
wind_std = 0.5

# --- Simulate ---
true_positions = []
gps_readings = []

position = 0.0
for step in range(num_steps):
    position = position + true_velocity * dt + np.random.normal(0, wind_std)
    true_positions.append(position)

    noisy_reading = position + np.random.normal(0, gps_noise_std)
    gps_readings.append(noisy_reading)

# --- Matrix Kalman filter setup ---
F = np.array([[1, dt],
              [0, 1]])             # physics: position += velocity * dt

H = np.array([[1, 0]])             # GPS only sees position

Q = np.array([[wind_std ** 2, 0],
              [0, 0.01]])          # how much uncertainty grows each step

R = np.array([[gps_noise_std ** 2]])   # GPS variance (9.0)

state = np.array([0.0, 0.0])       # start: position 0, velocity UNKNOWN (guess 0)

P = np.array([[1.0, 0.0],
              [0.0, 10.0]])        # pretty sure of position, very unsure of velocity

I = np.eye(2)                      # identity matrix (explained below)

# --- Matrix Kalman filter ---
position_estimates = []
velocity_estimates = []

for gps in gps_readings:
    # Predict
    state = F @ state
    P = F @ P @ F.T + Q

    # Update
    surprise = gps - H @ state
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    state = state + K @ surprise
    P = (I - K @ H) @ P

    position_estimates.append(state[0])
    velocity_estimates.append(state[1])

# --- Measure accuracy ---
true_arr = np.array(true_positions)
gps_arr = np.array(gps_readings)
est_arr = np.array(position_estimates)

gps_rmse = np.sqrt(np.mean((gps_arr - true_arr) ** 2))
kalman_rmse = np.sqrt(np.mean((est_arr - true_arr) ** 2))

print(f"Raw GPS error:        {gps_rmse:.2f} m")
print(f"Matrix Kalman error:  {kalman_rmse:.2f} m")
print(f"Final velocity guess: {velocity_estimates[-1]:.2f} m/s (true: {true_velocity} m/s)")

# --- Plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

ax1.plot(true_positions, label="True position", linewidth=2)
ax1.scatter(range(num_steps), gps_readings, color="red", s=12, label="GPS readings")
ax1.plot(position_estimates, label="Matrix Kalman filter", color="purple", linewidth=2)
ax1.set_xlabel("Time step (seconds)")
ax1.set_ylabel("Position (meters)")
ax1.set_title("Position tracking")
ax1.legend()

ax2.plot(velocity_estimates, label="Estimated velocity", color="purple", linewidth=2)
ax2.axhline(true_velocity, color="blue", linestyle="--", label="True velocity")
ax2.set_xlabel("Time step (seconds)")
ax2.set_ylabel("Velocity (m/s)")
ax2.set_title("Velocity learned from GPS alone")
ax2.legend()

plt.tight_layout()
plt.savefig("results_matrix.png", dpi=150, bbox_inches="tight")
plt.show()