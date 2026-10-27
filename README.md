# Kalman Drone Tracker

Tracking a simulated drone's position from noisy GPS data using a Kalman filter, built from scratch in Python with NumPy and Matplotlib.

> 🚧 **Work in progress.** The simulation and prediction step are done; the full Kalman filter is next.

## The problem

A drone has two imperfect ways to know where it is:

- **Physics prediction**: "I was at 10 m moving 2 m/s, so now I'm at 12 m." Smooth, but small errors (like wind) pile up and it slowly drifts off.
- **GPS**: Never drifts, but every reading is noisy and jumps around by several meters.

They fail in opposite ways. A Kalman filter blends the two, trusting whichever is more reliable at each moment, to get an estimate better than either one alone.

## What's built so far

- **Simulation**: a drone flying in a straight line, pushed around by random wind (process noise), with GPS readings that include random error (measurement noise).
- **Predict-only tracker**: estimates position using physics alone, showing how it drifts away from the true path over time.

## Coming next

- [ ] Update step: blend predictions with GPS using the Kalman gain
- [ ] Track estimate uncertainty over time
- [ ] Measure accuracy (error vs. raw GPS)
- [ ] Extend from 1D to 2D tracking

## How to run

```bash
conda create -n kalman python=3.11 numpy matplotlib -y
conda activate kalman
python simulate.py
```

## Tech

Python, NumPy, Matplotlib