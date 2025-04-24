import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# timestamp, alt, pres, a_x, a_y, a_z, g_x, g_y, g_z, temp

def estimate_attitude(ax, ay, az):
  """
  Estimate pitch and roll from accelerometer data.
  Assumes ax, ay, az are in units of g (gravitational acceleration).
  Returns pitch (in degrees) and roll (in degrees).
  """
  pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * (180 / np.pi)
  roll = np.arctan2(ay, az) * (180 / np.pi)
  return pitch, roll

def estimate_altitude_from_pressure(pres, sea_level_pres=1013.25):
  """
  Estimate altitude from barometric pressure using the barometric formula.
  Assumes pressure is in hPa and sea_level_pres is the sea-level pressure in hPa.
  Returns altitude in meters.
  """
  altitude = 44330 * (1 - (pres / sea_level_pres) ** (1 / 5.255))
  return altitude

def estimate_altitude_from_acceleration(ax, ay, az, g=9.81):
  """
  Estimate altitude from vertical acceleration data.
  Assumes ax, ay, az are in m/s^2 and g is the gravitational acceleration in m/s^2.
  Returns cumulative altitude in meters.
  """
  vertical_acceleration = az - g  # Subtract gravity to get net vertical acceleration
  dt = np.gradient(data['timestamp'])  # Time intervals
  velocity = np.cumsum(vertical_acceleration * dt)  # Integrate acceleration to get velocity
  altitude = np.cumsum(velocity * dt)  # Integrate velocity to get altitude
  return velocity

# Read CSV file
file_path = "/home/david/Documents/Rocketry/Rockety1/data_plot/4_4_1.CSV"  # Replace with your CSV file path
data = pd.read_csv(file_path)
apogee_idx = data['alt'].idxmax()

# Filter the data up to (but not including) apogee
data = data.iloc[:apogee_idx]
data['timestamp'] = (data['timestamp'] - data['timestamp'].min())/1000
print(data['timestamp'])
data = data[data['timestamp'] > 130]
data = data[data['timestamp'] < 151]
data['timestamp'] = (data['timestamp'] - data['timestamp'].min())

timestamp = data['timestamp']

print(data.columns)
ax = data['a_x']
ay = data['a_y']
az = data['a_z']# Extract accelerometer data

# c_ax = data[data['timestamp'] < 280]['a_x']
# c_ay = data[data['timestamp'] < 280]['a_y']
# c_az = data[data['timestamp'] < 280]['a_z']

# print(np.average(c_ax))
# print(np.average(c_ay))
# print(9.81-np.average(c_az))
az += (0.09688717948717951)



pres = data['pres']

# Estimate attitudes
pitch, roll = estimate_attitude(ax, ay, az)

# Estimate altitude from pressure
alt = altitude_from_pressure = data['alt'].to_numpy()
# Estimate altitude from acceleration
altitude_from_acceleration = estimate_altitude_from_acceleration(ax, ay, az)

time = data['timestamp'].to_numpy()

vel = np.zeros_like(alt)
for i in range(1, len(alt)):
    dt = time[i] - time[i-1]
    if dt > 0:
        vel[i] = (alt[i] - alt[i-1]) / dt
    else:
        vel[i] = vel[i-1]  # fallback in case of 0 time diff

# 2. Outlier removal: clamp extreme velocities (manual threshold)
max_change = 100  # max vertical speed in m/s, tune as needed
for i in range(len(vel)):
    if abs(vel[i]) > max_change:
        vel[i] = vel[i-1]  # simple clamping

# 3. Low-pass filter: exponential moving average
alpha = 0.1  # smoothing factor (between 0 and 1), smaller = smoother
vel_smoothed = np.zeros_like(vel)
vel_smoothed[0] = vel[0]
for i in range(1, len(vel)):
    vel_smoothed[i] = alpha * vel[i] + (1 - alpha) * vel_smoothed[i-1]

# Save to dataframe
data['vel_from_alt'] = vel_smoothed

# Plot the results
plt.figure(figsize=(12, 10))

# Plot pitch and roll
plt.subplot(3, 1, 1)
plt.plot(data['timestamp'], pitch, label='Pitch (degrees)', color='blue')
plt.plot(data['timestamp'], roll, label='Roll (degrees)', color='orange')
plt.title('Estimated Attitudes from Accelerometer Data')
plt.xlabel('Timestamp')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid()

# Plot altitude from pressure
plt.subplot(3, 1, 2)
print(np.max(altitude_from_pressure))
plt.plot(data['timestamp'], altitude_from_pressure, label='Altitude from Pressure (meters)', color='green', marker='o', markersize=6)
plt.title('Estimated Altitude from Barometric Pressure')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (meters)')
plt.legend()
plt.grid()

# Plot altitude from acceleration 
plt.subplot(3, 1, 3)

# print(alt_fused)
plt.plot(data['timestamp'], data['vel_from_alt'] , label='velocity', color='red', marker='o', markersize=6)
plt.title('Estimated Altitude from Accelerometer Data')
plt.xlabel('Timestamp')
plt.ylabel('Velocity (meters)')
plt.legend()
plt.grid()

apogee = alt.max()

# 2. Compute altitude-to-apogee
data['alt_to_apogee'] = apogee - alt

# 3. Plot
plt.figure(figsize=(8, 5))
plt.plot(vel, data['alt_to_apogee'], marker='o', linestyle='-', linewidth=1)
plt.xlabel("Altitude to Apogee (m)")
plt.ylabel("Vertical Velocity (m/s)")
plt.title("Altitude to Apogee vs Velocity")
plt.grid(True)
plt.tight_layout()
plt.show()