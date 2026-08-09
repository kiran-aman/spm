#!/usr/bin/env python3

# GENERATE PLOT USED FOR EXTENDED ABSTRACT IROS2026 FIGURE 3
# 180 DEGREE ROLL: DESIRED VS ACTUAL ORIENTATION
# FROM FILE 180yaw.csv

import argparse
import csv
import re
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

euler_rows = []
with open("180yaw.csv", newline="") as f:
    r = csv.DictReader(f)
    for row in r:
        euler_rows.append((float(row["t_rel_s"]), float(row["roll_des_deg"]), float(row["pitch_des_deg"]), float(row["yaw_des_deg"]),
                           float(row["roll_act_deg"]), float(row["pitch_act_deg"]), float(row["yaw_act_deg"])))

ts = np.array([r[0] for r in euler_rows])

roll_d  = np.array([r[1] for r in euler_rows])
pitch_d = np.array([r[2] for r in euler_rows])
yaw_d   = np.array([r[3] for r in euler_rows])
roll_a  = np.array([r[4] for r in euler_rows])
pitch_a = np.array([r[5] for r in euler_rows])
yaw_a   = np.array([r[6] for r in euler_rows])

fig, ax = plt.subplots(figsize=(10, 5))

ax.plot(ts, roll_d, label="Desired Roll", color="red")
ax.plot(ts, roll_a, "--", label="Actual Roll", color="#F14637")
ax.set_ylabel("Angle (deg)", fontsize=15)
ax.grid(True)
ax.plot(ts, pitch_d, label="Desired Pitch", color="green")
ax.plot(ts, pitch_a, "--", label="Actual Pitch", color="#5FE471")
ax.grid(True)
ax.plot(ts, yaw_d, label="Desired Yaw", color="blue")
ax.plot(ts, yaw_a, "--", label="Actual Yaw", color="#35A5F0")
ax.set_xlabel("Time (s)", fontsize=15)
ax.grid(True)
ax.legend(fontsize=15)
fig.tight_layout()
fig.savefig("180yaw.png", dpi=250)