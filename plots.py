# plots.py
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv("sim_log_PD_PID.csv")

# 1. Distance vs Time
plt.figure()
plt.plot(df["t"], df["dist"], label="Distance Error")
plt.xlabel("Time [s]")
plt.ylabel("Distance to Target [m]")
plt.title("Distance vs Time")
plt.grid(True)
plt.legend()
plt.savefig("distance_vs_time.png")
plt.show()

# 2. Heading Error vs Time
plt.figure()
plt.plot(df["t"], df["heading_err"], label="Heading Error", color="r")
plt.xlabel("Time [s]")
plt.ylabel("Heading Error [rad]")
plt.title("Heading Error vs Time")
plt.grid(True)
plt.legend()
plt.savefig("heading_error_vs_time.png")
plt.show()

# 3. Trajectory (x,y)
plt.figure()
plt.plot(df["x"], df["y"], label="Trajectory", color="g")
plt.scatter([df["x"].iloc[0]], [df["y"].iloc[0]], color="blue", label="Start")
plt.scatter([df["x"].iloc[-1]], [df["y"].iloc[-1]], color="red", label="End")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectory")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.savefig("trajectory.png")
plt.show()
