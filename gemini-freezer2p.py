import serial
import time
import RPi.GPIO as GPIO
from datetime import date
import matplotlib.pyplot as plt

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
#GPIO_PIN = 18
TARGET_TEMP = -20.0
CYCLE_TIME = 1800
ON_TIME_LIMIT = 150 # Max ON time within a cycle

# --- PID Controller Class ---
class PID:
    def __init__(self, P=2., I=1., D=1., current_time=None):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = current_time if current_time is not None else time.time()

    def update(self, feedback_value, current_time=None):
        current_time = current_time if current_time is not None else time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
        
        error = self.set_point - feedback_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.last_error = error
        self.last_time = current_time
        
        # Clamp output to a range appropriate for duty cycle (0 to 1 for 0% to 100% on time)
        # For a simple ON/OFF control using a relay, the output will be used to determine the *duration*
        # of the "ON" state within the allowed window.
        return max(0, min(ON_TIME_LIMIT, output)) # Max duration is the on_time_limit

# --- Setup Serial and GPIO ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # Timeout is important
    ser.flushInput()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)#, initial=GPIO.LOW)

pid = PID(P=10, I=0.1, D=0.01) # Tune Kp, Ki, Kd values for your specific freezer system
pid.set_point = TARGET_TEMP

print(f"Freezer control script started. Target temp: {TARGET_TEMP}°C")

fn = "Gemini_" + str(date.today()) + time.strftime("_H%H_M%M_S%S", time.localtime()) + ".csv"
f=open(fn, 'w', encoding="utf-8")
start=time.time()

plt.figure(100)
y1=[0]*100
x = range(0, 100, 1)

# --- Main Control Loop ---
try:
    while True:
        cycle_start_time = time.time()
        cycle_end_time = cycle_start_time + CYCLE_TIME
        on_duration = 0
        
        # (1-1) Control phase (0s to 1500s)
        while time.time() < cycle_start_time + ON_TIME_LIMIT:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line:
                    parts = line.split(',')
                    if parts[0] == '03' and len(parts) > 10:
                        current_temp = float(parts[3]) # Use data1 as temperature
                        
                        # Calculate PID output (which is now a duration or a percentage)
                        # For simple on-off control we need to map PID output to a duty cycle or on-time
                        # This example assumes the PID output directly maps to a percentage of the ON_TIME_LIMIT, 
                        # but a more robust method uses PWM or calculated pulse duration
                        
                        # A simple ON/OFF control logic based on error is often better for relays than direct PID output mapping to duration within a short window.
                        # Since a fixed cycle time is requested, we use the PID value to determine the 'on' duty cycle.
                        
                        # For simplicity, let's use a manual hysteresis for on-off, or a simple P control for duty cycle adjustment.
                        # The PID value here will be used to decide if the freezer runs during the next time slice.
                        
                        # A better approach for the specified cycle is using a calculated ON duration within the 1500s window.
                        # We need the PID library to output a value (e.g. 0 to 100%) and calculate the time.

                        # --- Simple On/Off Logic for Demonstration ---
                        if current_temp > TARGET_TEMP:
                            GPIO.output(18,1) # Turn freezer ON
                            print(f"Temp: {current_temp}°C (Above target). Freezer ON.")
                            ssr18=1
                            time.sleep(5)
                        else:
                            GPIO.output(18,0) # Turn freezer OFF
                            ssr18=0
                            time.sleep(5)
                            print(f"Temp: {current_temp}°C (Below target or at target). Freezer OFF.")
                        st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
                        ss = str(time.time() - int(time.time()))
                        sss=str(round(time.time()-start,2))
                        row=st + ss[1:5] + "," + sss + ","
                        row=row+str(current_temp)+","+str(ssr18)+"\n"
                        f.write(row)
                        title=str(round(time.time()-start,2))+',current_temp='+str(round(current_temp,2))+','+"ssr18="+str(ssr18)
                        plt.clf()
                        plt.title(title)
                        y1.pop(-1)
                        y1.insert(0,current_temp)
                        plt.ylim(-30,20)
                        plt.plot(x,y1)
                        plt.pause(0.1)
            except (ValueError, IndexError) as e:
                print(f"Error parsing serial data: {e}, Data: {line}")
            except serial.SerialException:
                print("Serial port error, re-establishing connection...")
                ser.close()
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            
#            time.sleep(5) # Small delay to prevent busy looping and allow other processes

        # (1-2) Off phase (1500s to 1800s)
        GPIO.output(18,0) # Ensure freezer is OFF during the mandated off cycle
        ssr18=0
        print("Mandatory OFF cycle (1500s-1800s). Freezer OFF.")
        while time.time() < cycle_end_time:
            time.sleep(5) # Sleep for the remainder of the cycle
            line = ser.readline().decode('utf-8').rstrip()
            parts = line.split(',')
            if parts[0] == '03' and len(parts) > 10:
              current_temp = float(parts[3])
              st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
              ss = str(time.time() - int(time.time()))
              sss=str(round(time.time()-start,2))
              row=st + ss[1:5] + "," + sss + ","
              row=row+str(current_temp)+","+str(ssr18)+"\n"
              f.write(row)
              title=str(round(time.time()-start,2))+',current_temp='+str(round(current_temp,2))+','+"ssr18="+str(ssr18)
              plt.clf()
              plt.title(title)
              y1.pop(-1)
              y1.insert(0,current_temp)
              plt.ylim(-30,20)
              plt.plot(x,y1)
              plt.pause(0.1)
#
        # (2) Next operation cycle
        print("Cycle complete. Starting new cycle.")

except KeyboardInterrupt:
    print("Script terminated by user.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    GPIO.cleanup()
    ser.close()
    f.close()
    print("GPIO cleaned up and serial port closed.")
