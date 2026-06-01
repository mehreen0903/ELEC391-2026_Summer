from serial import Serial
import matplotlib.pyplot as plt
import time

ser = Serial('COM5', 9600)

plt.ion() # interactive plotting

fig, ax = plt.subplots() # Create graph

ax.set_title('Accelerometer vs Gyroscope vs Comp Filter Tilt')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (Degrees)')

# Set fixed y-axis range
ax.set_ylim(-90, 90)

# Store graph data
x_data = []
acc_data = []   # Accelerometer angle
gyro_data = []  # gyroscope angle (gyro + acc ref)
comp_data = []  # complementary filter angle

# Create three lines with labels
line_acc, = ax.plot(x_data, acc_data, label='Accelerometer', color='blue')
line_gyro, = ax.plot(x_data, gyro_data, label='Gyro (Complementary)', color='orange')
line_comp, = ax.plot(x_data, comp_data, label='Complementary Filter', color='green')

ax.legend()

start_time = time.time()  # Record start time

while True:
    try:
        # Read serial data
        data = ser.readline().decode().strip()

        # Expect three comma-separated values: "accel_angle,gyro_angle,comp_angle"
        parts = data.split(',')
        if len(parts) != 3:
            continue

        acc_angle = float(parts[0])
        gyro_angle = float(parts[1])
        comp_angle = float(parts[2])

        elapsed = time.time() - start_time  # Seconds since start

        # Add new data points
        x_data.append(elapsed)
        acc_data.append(acc_angle)
        gyro_data.append(gyro_angle)
        comp_data.append(comp_angle)

        # Update all three graph lines
        line_acc.set_data(x_data, acc_data)
        line_gyro.set_data(x_data, gyro_data)
        line_comp.set_data(x_data, comp_data)

        # Move x-axis as data grows
        ax.set_xlim(0, elapsed + 1)

        # Redraw graph
        fig.canvas.draw()
        fig.canvas.flush_events()

    except KeyboardInterrupt:
        ser.close()
        break