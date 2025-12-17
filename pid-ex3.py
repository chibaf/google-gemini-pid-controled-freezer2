import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import date
import matplotlib.pyplot as plt

TARGET_TEMP = -20.0 # Target temperature in Celsius
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
GPIO_PIN = 18       # BCM pin number (GPIO 18)
CYCLE_TIME = 1800   # Total cycle time in seconds (30 minutes)
PID_CONTROL_DURATION = 1500 # PID control phase duration in seconds (25 minutes)
OFF_DURATION = 300  # Mandatory off phase duration in seconds (5 minutes)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
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
                    print(temperature)
                    return temperature
                except ValueError:
                    print(f"Bad data format or conversion error in data2: {parts[2]}")
    except Exception as e:
        print(f"Serial reading error: {e}")
    return [0,0]


def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    error = setpoint - pv
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral
def main():
    setpoint = -20.0  # Desired setpoint
    pv = 0  # Initial process variable
    kp = 1.0  # Proportional gain
    ki = 0.1  # Integral gain
    kd = 0.05  # Derivative gain
    previous_error = 10
    integral = 0
    dt = 0.1  # Time step
    time_steps = []
    pv_values = []
    control_values = []
    setpoint_values = []
    i=0
#    for i in range(100):  # Simulate for 100 time steps
# GPIO Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(18, GPIO.OUT)
    GPIO.output(18,0) # Ensure freezer starts OFF
    TARGET_TEMP = -20.0 # Target temperature in Celsius
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUDRATE = 115200
    GPIO_PIN = 18       # BCM pin number (GPIO 18)
    CYCLE_TIME = 1800   # Total cycle time in seconds (30 minutes)
    PID_CONTROL_DURATION = 1500 # PID control phase duration in seconds (25 minutes)
    OFF_DURATION = 300  # Mandatory off phase duration in seconds (5 minutes)
    try:
      ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
      time.sleep(2) # Wait for serial connection to establish
      ser.flushInput()
    except serial.SerialException as e:
      print(f"Error opening serial port: {e}")
      exit()

    time0=time.time()
    while 1:
        now=time.time()
        temp=read_temperature()
        pv=temp[1]
        i=i+1
        time.sleep(1)
        control, error, integral = pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt)
        pv += control * dt  # Update process variable based on control output (simplified)
        previous_error = error

        if 0<=(now-time0)<1500:
          if error<0:
            GPIO.output(GPIO_PIN,1)
          else:
            GPIO.output(GPIO_PIN,0)
        elif 1500<=(now-time0)<=1800:
          GPIO.output(GPIO_PIN,0)
        else:
          time0=time.time()
        time_steps.append(i * dt)
        pv_values.append(pv)
        control_values.append(control)
        setpoint_values.append(setpoint)
        print(str(error)+": "+str(i))
#        plt.clf()
#        time.sleep(dt)
#        plt.figure(figsize=(12, 6))
##        plt.title(str(i))
#        plt.subplot(2, 1, 1)
#        plt.plot(time_steps, pv_values, label='Process Variable (PV)')
#        plt.plot(time_steps, setpoint_values, label='Setpoint', linestyle='--')
#        plt.xlabel('Time (s)')
#        plt.ylabel('Value')
#        plt.title('Process Variable vs. Setpoint '+str(i))
#        plt.legend()
#
#        plt.subplot(2, 1, 2)
# #       plt.title(str(i))
#        plt.plot(time_steps, control_values, label='Control Output')
#        plt.xlabel('Time (s)')
#        plt.xlabel('Time (s)')
#        plt.ylabel('Control Output')
#        plt.title('Control Output over Time '+str(i))
#        plt.legend()
#
#        plt.tight_layout()
#        plt.show()        
#        plt.pause(0.1) 

if __name__ == "__main__":
    main()