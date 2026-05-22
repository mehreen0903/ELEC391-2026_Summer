from serial import Serial
import matplotlib.pyplot as plt
import time

# Connect to Arduino
ser = Serial('COM5', 9600)

# Turn on interactive plotting
plt.ion()

# Create graph
fig, ax = plt.subplots()

ax.set_title('Accelerometer vs Gyroscope Tilt')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (Degrees)')

# Set fixed y-axis range
ax.set_ylim(-90, 90)

# Store graph data
x_data = []
acc_data = []   # Accelerometer angle
gyro_data = []  # Complementary filter angle (gyro + acc)

# Create two lines with labels
line_acc, = ax.plot(x_data, acc_data, label='Accelerometer', color='blue')
line_gyro, = ax.plot(x_data, gyro_data, label='Gyro (Complementary)', color='orange')

ax.legend()

start_time = time.time()  # Record start time

while True:
    try:
        # Read serial data
        data = ser.readline().decode().strip()

        # Expect two comma-separated values: "accel_angle,gyro_angle"
        parts = data.split(',')
        if len(parts) != 2:
            continue

        acc_angle = float(parts[0])
        gyro_angle = float(parts[1])

        elapsed = time.time() - start_time  # Seconds since start

        # Add new data points
        x_data.append(elapsed)
        acc_data.append(acc_angle)
        gyro_data.append(gyro_angle)

        # Update both graph lines
        line_acc.set_data(x_data, acc_data)
        line_gyro.set_data(x_data, gyro_data)

        # Move x-axis as data grows
        ax.set_xlim(0, elapsed + 1)

        # Redraw graph
        fig.canvas.draw()
        fig.canvas.flush_events()

    except KeyboardInterrupt:
        ser.close()
        break