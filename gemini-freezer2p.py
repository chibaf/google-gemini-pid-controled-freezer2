import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import date
import matplotlib.pyplot as plt

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
GPIO_PIN = 18  # BCM pin number
TARGET_TEMP = -20.0  # degree C
CYCLE_DURATION = 1800  # seconds (30 minutes)
PID_CONTROL_DURATION = 1500  # seconds (25 minutes)
OFF_DURATION = 300  # seconds (5 minutes)

# PID parameters (these will need tuning for your specific freezer)
# Start with general values and adjust based on performance.
Kp = 10.0 #10
Ki = 0.5 # .5
Kd = 1.  # 1
SAMPLE_TIME = 10 # seconds

# --- Setup ---
# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.OUT)
GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer starts off

# Setup Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flushInput()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

fn = "Gemini_" + str(date.today()) + time.strftime("_H%H_M%M_S%S", time.localtime()) + ".csv"
f=open(fn, 'w', encoding="utf-8")
start=time.time()

plt.figure(100)
y1=[0]*100
x = range(0, 100, 1)

# Setup PID controller
pid = PID(Kp, Ki, Kd, setpoint=TARGET_TEMP)
# Set the output range. This will be the 'on time' percentage of a short cycle within the 1500s period.
# The output will range from 0 (off) to some max value (e.g., 100% on).
# We can normalize the PID output to a 0-100 range and then use that as a duty cycle.
pid.output_limits = (0, 100) 
pid.sample_time = SAMPLE_TIME # PID update rate

def read_temperature():
    """Reads the temperature from the serial port data stream."""
    try:
        line = ser.readline().decode('utf-8').rstrip()
        parts = line.split(',')
        if len(parts) >= 2 and parts[0] == '03':
            # Use data1 as temperature, convert to float
            temperature = float(parts[3])
            return temperature
    except (ValueError, IndexError, serial.SerialException) as e:
        print(f"Error processing serial data: {e}, Line: {line}")
    return None

def control_freezer(on_time_percent):
    """Controls the freezer on/off time based on PID output percentage over a short interval."""
    # We implement a simple PWM-like control here over the SAMPLE_TIME
    if on_time_percent >= 100: # 
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        time.sleep(SAMPLE_TIME)
    elif on_time_percent <= 0: # 
        GPIO.output(GPIO_PIN, GPIO.LOW)
        time.sleep(SAMPLE_TIME)
    else:
        on_duration = SAMPLE_TIME * (on_time_percent / 100.0)
        off_duration = SAMPLE_TIME * (1 - (on_time_percent / 100.0))
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        time.sleep(on_duration)
        GPIO.output(GPIO_PIN, GPIO.LOW)
        time.sleep(off_duration)

# --- Main Loop ---
try:
    while True:
        cycle_start_time = time.time()
        print(f"--- New {CYCLE_DURATION}s Cycle Started at {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
        
        # (1-1) From 0 to 1500 sec: PID control
        while (time.time() - cycle_start_time) < PID_CONTROL_DURATION:
            current_temp = read_temperature()
            if current_temp is not None:
                # Compute new PID output (percentage)
                pid_output_percent = pid(current_temp)
                print(f"Temp: {current_temp}°C, PID Output: {pid_output_percent:.1f}%")
                control_freezer(pid_output_percent) # control based on PID for SAMPLE_TIME duration
            else:
                # If no valid data, wait a bit and retry
                time.sleep(1)
            st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
            ss = str(time.time() - int(time.time()))
            sss=str(round(time.time()-start,2))
            row=st + ss[1:5] + "," + sss + ","
            row=row+str(current_temp)+"\n"
            f.write(row)
            title=str(round(time.time()-start,2))+',current_temp='+str(round(current_temp,2))
            plt.clf()
            plt.title(title)
            y1.pop(-1)
            y1.insert(0,current_temp)
            plt.ylim(-30,20)
            plt.plot(x,y1)
            plt.pause(0.1)

        # (1-2) From 1500 to 1800 sec: Switch off freezer
        print(f"--- PID control finished. Switching freezer OFF for {OFF_DURATION}s ---")
        GPIO.output(GPIO_PIN, GPIO.LOW)
#        time.sleep(OFF_DURATION)
        if (time.time() - cycle_start_time)<OFF_DURATION+PID_CONTROL_DURATION:
          st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
          ss = str(time.time() - int(time.time()))
          sss=str(round(time.time()-start,2))
          row=st + ss[1:5] + "," + sss + ","
          row=row+str(current_temp)+"\n"
          f.write(row)
          title=str(round(time.time()-start,2))+',current_temp='+str(round(current_temp,2))
          plt.clf()
          plt.title(title)
          y1.pop(-1)
          y1.insert(0,current_temp)
          plt.ylim(-30,20)
          plt.plot(x,y1)
          plt.pause(0.1)        
        # (2) Next operation cycle: go to (1)

except KeyboardInterrupt:
    print("Program terminated by user.")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is off on exit
    GPIO.cleanup()
    ser.close()
    f.close()
    print("GPIO cleaned up and serial port closed.")

