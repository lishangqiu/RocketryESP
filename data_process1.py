import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
file_path = "/home/david/Documents/Rocketry/Rockety1/data_plot/4_4_1.CSV"  # Replace with your CSV file path
data = pd.read_csv(file_path)

apogee_idx = data['alt'].idxmax()
# Filter the data up to (but not including) apogee
data = data.iloc[:apogee_idx]
data['timestamp'] = (data['timestamp'] - data['timestamp'].min()) / 1000
timestamp = data['timestamp']

# Plot all the data columns
for column in data.columns:
    if column != 'timestamp':  # Exclude timestamp from being plotted as y-axis
        plt.plot(timestamp, data[column], label=column)

plt.xlabel('Time (s)')
plt.ylabel('Values')
plt.title('Data Columns vs Time')
plt.legend()
plt.show()

CALM_START = 100
CALM_END = 130
calm_data = data[(timestamp >= CALM_START) & (timestamp <= CALM_END)]
calm_averages = calm_data.mean()

# Print the averages
print("Averages during the calm period:")
print(calm_averages)