import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import date
import matplotlib.pyplot as plt

# --- Configuration ---
TARGET_TEMP = -20.0 # Target temperature in Celsius
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
GPIO_PIN = 18       # BCM pin number (GPIO 18)
CYCLE_TIME = 1800   # Total cycle time in seconds (30 minutes)
PID_CONTROL_DURATION = 1500 # PID control phase duration in seconds (25 minutes)
OFF_DURATION = 300  # Mandatory off phase duration in seconds (5 minutes)

current_time = time.strftime("_H%H_M%M_S%S", time.localtime())
fn = "Gemini_" + str(date.today()) + current_time + ".csv"
f=open(fn, 'w', encoding="utf-8")
stime = time.time()

plt.figure(100)
y1=[0]*100
y2=[0]*100
x = range(0, 100, 1)
plt.ylim(-25,10)

# PID parameters (these will likely need tuning for your specific freezer)
# Start with general values and adjust as needed for stability and response.
KP = 5.0
KI = 1.0
KD = 1.0

# --- Initialize Hardware and PID ---

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.OUT)
GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer starts OFF

# Serial Setup
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2) # Wait for serial connection to establish
    ser.flushInput()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# PID Controller
pid = PID(KP, KI, KD, setpoint=TARGET_TEMP)
# We limit the PID output to a range suitable for time-proportional control (0 to 1)
# where 1 is full on time and 0 is full off time within the control phase.
pid.output_limits = (0.9,1)

# --- Functions ---

def read_temperature():
    """Reads and parses temperature data from the serial port."""
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            parts = line.split(',')
            if parts[0] == '03' and len(parts) > 10:
                try:
                    # data2 is at index 2 of the split list
                    temperature = [float(parts[2]),float(parts[3])]
                    #print(temperature)
                    return temperature
                except ValueError:
                    print(f"Bad data format or conversion error in data2: {parts[2]}")
    except Exception as e:
        print(f"Serial reading error: {e}")
    return None

def control_freezer(on_time, off_time):
    """Controls the freezer switch with specified on/off times."""
    if on_time > 0:
        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn freezer ON
        time.sleep(on_time)
    if off_time > 0:
        GPIO.output(GPIO_PIN, GPIO.LOW) # Turn freezer OFF
        time.sleep(off_time)

# --- Main Control Loop ---
cycle_start_time=time.time()-stime
try:
    while True:
        print("\n--- Starting New Operation Cycle ---")
        if time.time()-cycle_start_time>=CYCLE_TIME:
          cycle_start_time = time.time()
        
        while (time.time() - cycle_start_time) < PID_CONTROL_DURATION:
            temp = read_temperature()
            try:
              current_temp=temp[1]
            except Exception as e:
              continue
            if current_temp is not None:
                # Compute new control output (duty cycle proportion: 0.0 to 1.0)
                control_output = pid(current_temp)
                
                # Time-proportional control: calculate ON time based on PID output
                # We use a short cycle time here (e.g., 60 seconds) for responsiveness
                # within the larger 1500s window.
                sub_cycle_duration = 30 # seconds
                on_time_sub = control_output * sub_cycle_duration
                off_time_sub = sub_cycle_duration - on_time_sub
                
                print(f"Temp: {current_temp:.2f}C, PID Output: {control_output:.2f}, On Time: {on_time_sub:.2f}s, Off Time: {off_time_sub:.2f}s")
                st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
                ss = str(time.time() - int(time.time()))
                sss=str(round(time.time()-stime,2))
                row=st + ss[1:5] + "," + sss
                row=row+","+str(temp[0])+","+str(current_temp)+","+str(round(on_time_sub,2))+","+str(round(off_time_sub,2))+"\n"
                f.write(row)
                print(row)
# plotting
                title=str(round(time.time()-stime,2))+',temp1='+str(round(temp[0],2))+',current_temp='+str(round(current_temp,2))+","+"on time:"+str(round(on_time_sub,2))+",off time:"+str(round(off_time_sub,2))
                plt.clf()
                y1.pop(-1)
                y1.insert(0,temp[0])
                y2.pop(-1)
                y2.insert(0,current_temp)
                plt.title(title)
                plt.ylim(-25,20)
                plt.plot(x,y1)
                plt.plot(x,y2)
                plt.pause(0.1)
                # Act on the freezer for this sub-cycle
                control_freezer(on_time_sub, off_time_sub)
            else:
                # If no valid data is read, sleep briefly before next attempt
                time.sleep(5) 
            
            # Check if we are still within the 1500s control window
            if (time.time() - cycle_start_time) >= PID_CONTROL_DURATION:
              control_freezer(0, off_time_sub)
              temp = read_temperature()
              current_temp=temp[1]
              st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
              ss = str(time.time() - int(time.time()))
              sss=str(round(time.time()-stime,2))
              row=st + ss[1:5] + "," + sss
              row=row+","+str(temp[0])+","+str(current_temp)+","+str(round(on_time_sub,2))+","+str(round(off_time_sub,2))+"\n"
              f.write(row)
              print(row)                
              title=str(round(time.time()-stime,2))+',temp1='+str(round(temp[0],2))+',current_temp='+str(round(current_temp,2))+","+"on time:"+str(round(on_time_sub,2))+",off time:"+str(round(off_time_sub,2))
              plt.clf()
              y1.pop(-1)
              y1.insert(0,temp[0])
              y2.pop(-1)
              y2.insert(0,current_temp)
              plt.title(title)
              plt.ylim(-25,20)
              plt.plot(x,y1)
              plt.plot(x,y2)
              plt.pause(0.1)

        # Phase (1-2): Mandatory 300 sec OFF period
#        print(f"--- Entering Mandatory OFF Period ({OFF_DURATION}s) ---")
#        GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is OFF
#        time.sleep(OFF_DURATION)
        
        # Cycle completes, loop goes to (2) which returns to (1)

except KeyboardInterrupt:
    print("Program stopped by user. Cleaning up GPIO.")
finally:
    GPIO.output(GPIO_PIN, GPIO.LOW)
    GPIO.cleanup()
    ser.close()
    f.close()
