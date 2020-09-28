import matplotlib.pyplot as plt
import pickle
from mpl_toolkits import mplot3d

pickle_path = '/home/tello18/Pictures/sensor-data-path-2020-08-17_235612.pickle'

# load variables
x, y, z, x_sensor, y_sensor, z_sensor, temperature, humidity, co2 = pickle.load(file(pickle_path))

# Plot path
fig_path = plt.figure(1)
ax_path = fig_path.add_subplot(111, projection="3d")
ax_path.plot3D(x, y, z, 'blue', linewidth=1)
ax_path.set_xlabel('x in [m]')
ax_path.set_ylabel('y in [m]')
ax_path.set_zlabel('z in [m]')
ax_path.set_title('Flight path')
# fig_path.savefig(path + '.png')
plt.show()

fig_temperature = plt.figure(2)
ax_temperature = fig_temperature.add_subplot(111, projection="3d")
ax_temperature.plot3D(x, y, z, color='b', linewidth=1)
scatt = ax_temperature.scatter(x_sensor, y_sensor, z_sensor, c=temperature, cmap='gist_heat',
                               vmin=20, vmax=30, linewidths=3)
cbar = fig_temperature.colorbar(scatt, ax=ax_temperature, shrink=0.5, aspect=5)
ax_temperature.set_xlabel('x in [m]')
ax_temperature.set_ylabel('y in [m]')
ax_temperature.set_zlabel('z in [m]')
ax_temperature.set_title('Flight path with temperature')
cbar.set_label('Temperature in [$^\circ$C]', rotation=0, labelpad=0, y=-0.05)

plt.show()

fig_humidity = plt.figure(3)
ax_humidity = fig_humidity.add_subplot(111, projection="3d")
ax_humidity.plot3D(x, y, z, color='b', linewidth=1)
scatt = ax_humidity.scatter(x_sensor, y_sensor, z_sensor, c=humidity, cmap='YlGnBu', linewidths=3)
cbar = fig_humidity.colorbar(scatt, ax=ax_humidity, shrink=0.5, aspect=5)
ax_humidity.set_xlabel('x in [m]')
ax_humidity.set_ylabel('y in [m]')
ax_humidity.set_zlabel('z in [m]')
ax_humidity.set_title('Flight path with humidity')
cbar.set_label('Humidity in [%]', rotation=0, labelpad=0, y=-0.05)

plt.show()

fig_co2 = plt.figure(4)
ax_co2 = fig_co2.add_subplot(111, projection="3d")
ax_co2.plot3D(x, y, z, color='b', linewidth=1)
scatt = ax_co2.scatter(x_sensor, y_sensor, z_sensor, c=co2, cmap='plasma', linewidths=3)
cbar = fig_co2.colorbar(scatt, ax=ax_co2, shrink=0.5, aspect=5)
ax_co2.set_xlabel('x in [m]')
ax_co2.set_ylabel('y in [m]')
ax_co2.set_zlabel('z in [m]')
ax_co2.set_title('Flight path with CO2')
cbar.set_label('CO2 in [PPM]', rotation=0, labelpad=0, y=-0.05)

plt.show()


#self.ax_temperature.set_title('Temperature path')