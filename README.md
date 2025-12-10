# google-gemini-pid-controled-freezer2
google-gemini-pid-controled-freezer

<ul>
fix freezer’s temperature by PID control with python on Raspberry Pi<br>
(0-1) Target temperature: -20 degree C<br>
(0-2) input  data from serial<br>
   /dev/ttyUSB0, speed=115200/sec<br>
(0-3) record format from serial port<br>
    “03”,data0,data1,data2,data3,data4,data5,data6,data7,data8,data9,data10<br>
    we use data1 as temperature<br>
(1) operation cycle=1800sec    <br>
   (1-1) from 0 sec to 1500sec        control freezer switch via GPIO=18 using PID control<br>
   (1-2) from 1500sec to 1800sec <br>
     switch off freezer via GPIO=18<br>
(2) the next operation cycle: go to (1)<br>
</ul>
gemini: https://github.com/chibaf/google-gemini-pid-control-freezer/blob/main/gemini-freezer1.py

modified: https://github.com/chibaf/google-gemini-pid-control-freezer/blob/main/gemini-freezer1p.py

<img width="639" height="269" alt="image" src="https://github.com/user-attachments/assets/ebbe2332-cf7b-48ee-87cb-98116f263a8f" />
