import serial
import time
import RPi.GPIO as GPIO
import csv
import matplotlib.pyplot as plt
from collections import deque

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
GPIO_PIN = 18 # BCM pin numbering
TARGET_TEMP = -20.0
CYCLE_TIME = 1800 # seconds
CONTROL_WINDOW = 1500 # seconds (0 to 1500)
OFF_WINDOW = 300 # seconds (1500 to 1800)
LOG_FILE = 'freezer_log'+str(time.time())+'.csv'

# --- PID Controller Implementation (simplified) ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()

    def update(self, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0 # Avoid division by zero

        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        
        # PID output (basic version, needs tuning and potentially windup prevention)
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.previous_error = error
        self.last_time = current_time
        
        # Clamp output for a freezer (on/off control, not proportional power)
        # The output value will represent the duty cycle percentage (0 to 100) or similar
        # We will use this to determine "on time" within a smaller timeframe
        return max(0, min(100, output))

# Initialize PID with placeholder gains (these must be tuned for your specific system)
# Kp, Ki, Kd values depend heavily on the freezer's thermal dynamics
# Start with Kp, adjust Ki/Kd later.
pid = PIDController(Kp=5.0, Ki=0.1, Kd=0.5, setpoint=TARGET_TEMP)

# --- Hardware Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)
GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure the freezer starts off

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flushInput()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# --- Logging and Plotting Setup ---
def log_data(elapsed_time, temp, status):
    with open(LOG_FILE, 'a+', newline='') as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(['Timestamp', 'Elapsed Time (s)', 'Temperature (C)', 'Freezer Status'])
        writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), elapsed_time, temp, status])

temp_history = deque(maxlen=100)
plt.ion() # Turn on interactive plotting
fig, ax = plt.subplots()
line, = ax.plot(list(range(len(temp_history))), list(temp_history))
ax.set_ylim(TARGET_TEMP - 5, TARGET_TEMP + 5) # Adjust limits as needed
ax.set_xlabel("Data Point Index")
ax.set_ylabel("Temperature (°C)")
ax.set_title("Freezer Temperature over Time")

def update_plot():
    line.set_ydata(list(temp_history))
    line.set_xdata(list(range(len(temp_history))))
    ax.relim()
    ax.autoscale_view(True,True,True)
    fig.canvas.draw()
    fig.canvas.flush_events()

# --- Main Control Loop ---
cycle_start_time = time.time()

while True:
  try:
    elapsed_cycle_time = time.time() - cycle_start_time

    if elapsed_cycle_time >= CYCLE_TIME:
        cycle_start_time = time.time()
        elapsed_cycle_time = 0
#        ser.flushInput() # Clear serial buffer for new cycle

    # (0-2), (0-3) Read temperature from serial
    try:
        line_bytes = ser.readline()
        if line_bytes:
            line_str = line_bytes.decode('utf-8').strip()
            parts = line_str.split(',')
            if parts[0] == '03' and len(parts) > 3:
                current_temp_str = parts[3]
                try:
                    current_temp = float(current_temp_str)
                    temp_history.append(current_temp)

                    # (1-3) Log data
                    log_data(elapsed_cycle_time, current_temp, GPIO.input(GPIO_PIN))

                    # (1-4) Update plot every few steps (or adjust logic)
                    if len(temp_history) % 5 == 0:
                        update_plot()

                except ValueError:
                    print(f"Could not parse temperature from data: {parts[2]}")

    except serial.SerialException:
#        print("Serial communication error.")
#        ser.close()
#        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        continue
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        continue

    # (1-1) & (1-2) Control logic within the cycle
    if elapsed_cycle_time < CONTROL_WINDOW:
        # Compute PID output for on-time calculation (e.g., as a percentage/duration)
        pid_output = pid.update(current_temp) # Output is 0-100
        
        # Simple On-Off control using PID output to determine duration
        # This is basic pulse width modulation (PWM) implemented in software
        # The PID output determines the percentage of a small time slice the freezer is ON
        
        # Instead of calculating "on time" for the full 1500s at once,
        # we can use the PID output to adjust the freezer's state dynamically in a smaller window
        
        # A more practical approach for on/off control (bang-bang style) is often used
        # or calculate a duty cycle over a smaller period (e.g., every minute)

        # For this specific requirement (calculate "on/off time" for the window),
        # we treat PID output as a signal to turn on or off at the current moment
        if current_temp < TARGET_TEMP - 0.5: # Example threshold
             GPIO.output(GPIO_PIN, GPIO.LOW) # Turn off if too cold
        elif current_temp > TARGET_TEMP + 0.5: # Example threshold
             GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn on if too warm
        else:
             # Maintain current state or use PID output for more nuanced control
             pass

    else:
        # (1-2) From 1500s to 1800s, switch off freezer
        GPIO.output(GPIO_PIN, GPIO.LOW)

    time.sleep(1) # Small delay to prevent burning CPU, adjust as needed for control frequency

  except KeyboardInterrupt:
    print("Program terminated by user")
#  finally:
    GPIO.cleanup()
    ser.close()
#    print("GPIO pins reset and serial port closed.")
