from wsgiref.simple_server import make_server
from urllib.parse import parse_qs
from string import Template
import rtmidi
import json

cmd_map={'stop':1,'up':2,'down':3,'floor':4,'top':5,'rigtop':6,'stepup':10,'stepdown':11}


# WSGI application. Displays index.html with filled in MIDI ports
# Called with args it sends appropriate MIDI message to chosen MIDI device
def midi_control_app(environ, start_response):
    query_params = parse_qs(environ['QUERY_STRING'])
    status = '200 OK'
    ret=''
    channel=1
    programNumber=1
    rtm = rtmidi.RtMidiOut()

    ports=[]
    for i in range(rtm.getPortCount()):
        ports.append(rtm.getPortName(i))
    portss =",".join(f"'{item}'" for item in ports)
    print('MIDI:',ports, portss)

    if len(query_params) == 0:
        headers = [('Content-type', 'text/html')]
        with open('index.html') as file:
            ret = Template(file.read())
        ret = [bytes(ret.substitute(motors=portss),'ascii')]
    else:
        headers = [('Content-type', 'text/json; charset=utf-8')]
        motor_number = query_params.get('motor_number')
        command = query_params.get('command', [])
        print('query_params',json.dumps(query_params))
        ret = [bytes(json.dumps(query_params),'utf-8')]
        rtm.openPort(ports.index(motor_number[0]))
        prog_change_msg = rtmidi.MidiMessage.programChange(channel, cmd_map[command[0]])
        rtm.sendMessage(prog_change_msg)
    rtm.closePort()

    start_response(status, headers)

    return ret

with make_server('0.0.0.0', 8000, midi_control_app) as httpd:
    print("Serving on port 8000...")
    httpd.serve_forever()

