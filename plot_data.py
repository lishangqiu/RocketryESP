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
data['timestamp'] = (data['timestamp'] - data['timestamp'].min())/1000
# data = data[data['timestamp'] > 250]

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

a_vec = np.stack([ax, ay, az], axis=1)
stationary = data[data['timestamp'] < 280]
g_vec = np.array([stationary['a_x'].mean(),
                  stationary['a_y'].mean(),
                  stationary['a_z'].mean()])
g_unit = g_vec / np.linalg.norm(g_vec)

# Project each acceleration vector onto the gravity vector
a_proj_on_g = np.dot(a_vec, g_unit)  # scalar projection
a_vertical = a_proj_on_g - 9.81  # dynamic vertical acceleration

# Add to dataframe
data['a_vertical'] = a_vertical



alpha = 0.01  # tuning parameter (between 0 and 1)

# Assume `dt` is your time step (seconds), already computed
# Initialize altitude estimates
alt_acc = 0
alt_fused = [0,]

vel = 0
for i in range(1, len(data)):
    dt = data['timestamp'].iloc[i] - data['timestamp'].iloc[i-1]
    # Integrate acceleration twice to get altitude
    acc_z = data['a_vertical'].iloc[i]
    vel = vel + acc_z * dt
    alt_acc = alt_acc + vel * dt

    # Fuse the altitudes
    alt_baro = data['alt']
    # print(alt_acc)
    # print(alt_baro.iloc[i])
    fused = alpha * (alt_acc) + (1 - alpha) * alt_baro.iloc[i]
    # fused = data['alt'].iloc[i]
    alt_fused.append(fused)

# Extract pressure data
pres = data['pres']

# Estimate attitudes
pitch, roll = estimate_attitude(ax, ay, az)

# Estimate altitude from pressure
altitude_from_pressure = data['alt']

# Estimate altitude from acceleration
altitude_from_acceleration = estimate_altitude_from_acceleration(ax, ay, az)

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
plt.plot(data['timestamp'], altitude_from_pressure, label='Altitude from Pressure (meters)', color='green')
plt.title('Estimated Altitude from Barometric Pressure')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (meters)')
plt.legend()
plt.grid()

# Plot altitude from acceleration
plt.subplot(3, 1, 3)

# print(alt_fused)
plt.plot(data['timestamp'], alt_fused, label='Altitude from Acceleration (meters)', color='red')
plt.title('Estimated Altitude from Accelerometer Data')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (meters)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()