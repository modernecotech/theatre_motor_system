# theatre_motor_system
an ESP32 controlled motor for theatres and other automated systems requiring the syncronisation and control of both speed and direction of multiple motors using simple python scripts from the controller.


The main scripts are as follows. 

There is a "bluetooth" version of the ESP32 controller for use cases requiring a small number of motors where the user does not seek to have an Access Point. 

The main version is using the WiFi of the ESP32 and receives UDP broadcasts from the "controller" (in this instance its a simple setup using a spreadsheet to layout the "motor movement track" - for maybe 40 motors maximum? 

Finally a python script to send the codes by UDP to all the motors. 

the code to drive the motors is as follows. 

M01 - Motor number
D or U - "Down" (turning direction)
100 - Speed as an integer between 0 and 255 with 0 being stopped state. 

an example code 

M01D50 - turn motor 1 downwards at a speed of 50 
M02U100 - turn motor 2 up at a speed of 100 

etc... 
