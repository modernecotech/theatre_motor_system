import PySimpleGUI as sg
import serial
import serial.tools.list_ports
import time

# find the right serial port for usb wired connection
'''
comport = ''
for x in serial.tools.list_ports.comports():
    if "CH340" in x.description:
        comport = x.device
        ser = serial.Serial(port=comport,baudrate=9600)
'''

# using pandas to read in the file
import pandas as pd
df = pd.read_excel("Motor Movement Composer.ods", engine="odf", sheet_name="composer")

# using wifi UDP broadcast
import udp
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = '255.255.255.255'
port = 8080
udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# user interface layout

M01_SLIDER = [[sg.Text(text="M01")],
    [sg.Slider(range=(0,255), default_value=125, orientation='v', size=(8,30), enable_events=True, key="-M01-SPEED-", change_submits=True)]]
M01_DIRECTION = [[sg.Button("UP", key="M01U")],
                 [sg.Button("STOP", key="M01S")],
                 [sg.Button("DOWN", key="M01D")]]
M02_SLIDER = [[sg.Text(text="M02")],
    [sg.Slider(range=(0,255), default_value=125, orientation='v', size=(8,30), enable_events=True, key="-M02-SPEED-", change_submits=True)]]
M02_DIRECTION = [[sg.Button("UP", key="M02U")],
                 [sg.Button("STOP", key="M02S")],
                 [sg.Button("DOWN", key="M02D")]]
MOTOR_STARTER = [[sg.Button("START SEQUENCE", key="START_SEQUENCE")]]
MOTOR_STOP = [[sg.Button("STOP", key="STOP")]]
layout = [[sg.Column(M01_SLIDER, element_justification='c'), 
           sg.Column(M01_DIRECTION, element_justification='c'),
           sg.VSeperator(),
           sg.Column(M02_SLIDER, element_justification='c'),
           sg.Column(M02_DIRECTION, element_justification='c'),
           sg.VSeperator(),
           sg.Column(MOTOR_STARTER, element_justification='c'), 
           sg.Column(MOTOR_STOP, element_justification='c')]]
           

window = sg.Window('Theatre motors', layout, finalize=True)


def sequence_running(df):
    for index, row in df.iterrows():
        time.sleep(1)
        for x in row[1:]:
            for d in df.columns[1:]:
                if x >= 0:
                    print(f'{d}U{int(row[1])}')
                elif x < 0:
                    print(f'{d}D{int(row[1])}')
                else:
                    pass


for index, row in df.iterrows():
    for x in row[1:]:
        for d in df.columns[1:]:
            if x >= 0:
                print(f'{d}U{int(row[1])}')

    


while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        #command below for bluetooth
        # ser.write(b'STOP')
        
        #command below for wifi udp
        udp_sender.sendto(b'STOP', (host, port))
        break
    elif event == "START_SEQUENCE":
        print("starting sequence")
    elif event == "STOP":
        print("stop sequence")
        array = str.encode(event)
        udp_sender.sendto(array, (host, port))
    else:
        array = str.encode(event)
        # command below for bluetooth
        # ser.write(array)

        # command below for wifi udp
        udp_sender.sendto(array, (host, port))

window.close()
