import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Serial Port Configuration ---
SERIAL_PORT = 'COM4'    # Change based on your OS (e.g., "/dev/ttyUSB0" for Linux)
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    exit(1)

# Store x, y positions
x_data, y_data = [], []

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)  # Adjust based on your workspace
ax.set_ylim(-5, 5)
ax.set_title("Real-Time Robot Path")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
line, = ax.plot([], [], 'ro-', markersize=5)  # Red circles for the path

def update_plot(frame):
    global x_data, y_data
    
    try:
        raw_line = ser.readline().decode(errors='ignore').strip()  # Ignore decode errors
        if raw_line:
            parts = raw_line.split(',')

            # Ensure we have exactly 2 float values
            if len(parts) == 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    x_data.append(x)
                    y_data.append(y)

                    # Update plot
                    line.set_data(x_data, y_data)
                    ax.set_xlim(min(x_data)-0.5, max(x_data)+0.5)
                    ax.set_ylim(min(y_data)-0.5, max(y_data)+0.5)

                except ValueError:
                    pass  # Ignore lines that can't be converted to float

    except Exception as e:
        print("Error:", e)

ani = animation.FuncAnimation(fig, update_plot, interval=100, save_count=100)  # Limit frames saved

plt.show()
