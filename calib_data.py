import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
file_path = "/home/david/Documents/Rocketry/Rockety1/data_plot/calibration.csv"  # Replace with your CSV file path
data = pd.read_csv(file_path)
print(data.columns)
# Extract accelerometer data
ax = data['a_x']
ay = data['a_y']
az = data['a_z']
gx = data['g_x']
gy = data['g_y']
gz = data['g_z']
print(np.average(ax), np.average(ay), np.average(az), np.average(gx), np.average(gy), np.average(gz))