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
    fn = "Ex3_" + str(date.today()) + time.strftime("_H%H_M%M_S%S", time.localtime()) + ".csv"
    f=open(fn, 'w', encoding="utf-8")
    start=time.time()
    setpoint = -20.0  # Desired setpoint
    pv = 0  # Initial process variable
    kp = 2.0  # Proportional gain
    ki = 0.  # Integral gain
    kd = 0.  # Derivative gain
    previous_error = 10
    integral = 0
    dt = 0.1  # Time step
#    time_steps = []
#    pv_values = []
#    control_values = []
#    setpoint_values = []
    x=range(0,100)
    y1=[0]*100
    y2=[0]*100
#
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
        try:
          if temp[0]==0 and temp[1]==0:
            continue
        except:
          continue
        pv=temp[1]
        i=i+1
        time.sleep(1)
        control, error, integral = pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt)
        print(control, error, integral)
        pv += control * dt  # Update process variable based on control output (simplified)
        previous_error = error
        ssr18=""
        if 0<=(now-time0)<1500:
          if error<0.:
            ssr18="1"
            GPIO.output(GPIO_PIN,1)
          else:
            ssr18="0"
            GPIO.output(GPIO_PIN,0)
        elif 1500<=(now-time0)<=1800:
          ssr18="0"
          GPIO.output(GPIO_PIN,0)
        else:
          time0=time.time()
#        time_steps.append(i * dt)
#        pv_values.append(pv)
#        control_values.append(control)
#        setpoint_values.append(setpoint)
        print(str(error)+": "+str(i))
        st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
        ss = str(time.time() - int(time.time()))
        sss=str(round(time.time()-start,2))
        row=st + ss[1:5] + "," + sss + ","
        row=row+str(temp[0])+","+str(temp[1])+","+str(ssr18)+"\n"
        f.write(row)
        ttl="time="+str(round(now-time0,2))+",ssr18="+str(ssr18)+",temp1="+str(round(temp[0],2))+",temp2="+str(round(temp[1],2))
        plt.clf()
        y1.pop(-1)
        y1.insert(0,temp[0])
        y2.pop(-1)
        y2.insert(0,temp[1])
        plt.ylim(-30,20)
        plt.title(ttl)
        plt.plot(x,y1)
        plt.plot(x,y2)
        plt.pause(0.1)
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