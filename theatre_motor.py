import PySimpleGUI as sg
import serial
import serial.tools.list_ports

# find the right serial port for usb wired connection
'''
comport = ''
for x in serial.tools.list_ports.comports():
    if "CH340" in x.description:
        comport = x.device
        ser = serial.Serial(port=comport,baudrate=9600)
'''

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
layout = [[sg.Column(M01_SLIDER, element_justification='c'), 
           sg.Column(M01_DIRECTION, element_justification='c'),
           sg.VSeperator(),
           sg.Column(M02_SLIDER, element_justification='c'),
           sg.Column(M02_DIRECTION, element_justification='c')]]

window = sg.Window('Theatre motors', layout, finalize=True)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        #command below for bluetooth
        # ser.write(b'STOP')
        
        #command below for wifi udp
        udp_sender.sendto(b'STOP', (host, port))
        break
    else:
        array = str.encode(event)
        # command below for bluetooth
        # ser.write(array)

        # command below for wifi udp
        udp_sender.sendto(array, (host, port))

window.close()