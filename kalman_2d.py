import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# --Settings--
dt = 1.0
num_steps = 60
speed = 2.0            # m/s
turn_rate = 0.05       # how fast the drone curves (radians per second)
gps_noise_std = 3.0
wind_std = 0.5

# --Simulate a curved flight--
true_x, true_y = [], []
gps_x, gps_y = [], []

x, y = 0.0, 0.0
heading = 0.0          # direction the drone is pointing (radians)

for step in range(num_steps):
    heading = heading + turn_rate * dt
    x = x + speed * np.cos(heading) * dt + np.random.normal(0, wind_std)
    y = y + speed * np.sin(heading) * dt + np.random.normal(0, wind_std)
    true_x.append(x)
    true_y.append(y)

    gps_x.append(x + np.random.normal(0, gps_noise_std))
    gps_y.append(y + np.random.normal(0, gps_noise_std))

# --2D Kalman filter setup--
F = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1,  0],
              [0, 0, 0,  1]])

H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

Q = np.diag([wind_std**2, wind_std**2, 0.1, 0.1])
R = np.diag([gps_noise_std**2, gps_noise_std**2])

state = np.array([0.0, 0.0, 0.0, 0.0])   # [x, y, vx, vy]
P = np.diag([1.0, 1.0, 10.0, 10.0])
I = np.eye(4)

# --2D Kalman filter--
est_x, est_y = [], []

for gx, gy in zip(gps_x, gps_y):
    gps = np.array([gx, gy])

    # Predict
    state = F @ state
    P = F @ P @ F.T + Q

    # Update
    surprise = gps - H @ state
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    state = state + K @ surprise
    P = (I - K @ H) @ P

    est_x.append(state[0])
    est_y.append(state[1])

# --- Measure accuracy (distance error) ---
tx, ty = np.array(true_x), np.array(true_y)
gx_arr, gy_arr = np.array(gps_x), np.array(gps_y)
ex_arr, ey_arr = np.array(est_x), np.array(est_y)

gps_rmse = np.sqrt(np.mean((gx_arr - tx)**2 + (gy_arr - ty)**2))
kalman_rmse = np.sqrt(np.mean((ex_arr - tx)**2 + (ey_arr - ty)**2))

print(f"Raw GPS error:    {gps_rmse:.2f} m")
print(f"2D Kalman error:  {kalman_rmse:.2f} m")
print(f"Improvement:      {(1 - kalman_rmse / gps_rmse) * 100:.0f}%")

# --- Plot ---
plt.figure(figsize=(8, 8))
plt.plot(true_x, true_y, label="True path", linewidth=2)
plt.scatter(gps_x, gps_y, color="red", s=12, label="GPS readings")
plt.plot(est_x, est_y, label="2D Kalman filter", color="purple", linewidth=2)
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("2D drone tracking from noisy GPS")
plt.axis("equal")
plt.legend()
plt.savefig("results_2d.png", dpi=150, bbox_inches="tight")
plt.show()