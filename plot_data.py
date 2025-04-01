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
  return altitude

# Read CSV file
file_path = "./data_plot/LOGGER11.csv"  # Replace with your CSV file path
data = pd.read_csv(file_path)
print(data.columns)
# Extract accelerometer data
ax = data['a_x']
ay = data['a_y']
az = data['a_z']
data['timestamp'] = (data['timestamp'] - data['timestamp'].min())/1000

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
plt.plot(data['timestamp'], altitude_from_pressure, label='Altitude from Pressure (meters)', color='green')
plt.title('Estimated Altitude from Barometric Pressure')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (meters)')
plt.legend()
plt.grid()

# Plot altitude from acceleration
plt.subplot(3, 1, 3)
plt.plot(data['timestamp'], altitude_from_acceleration, label='Altitude from Acceleration (meters)', color='red')
plt.title('Estimated Altitude from Accelerometer Data')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (meters)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()