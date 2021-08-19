from __future__ import  print_function, division, absolute_import
import serial.tools.list_ports
import serial
import time

BAUD_RATE = 57600

class ArduinoConnection(object):
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=BAUD_RATE, timeout=0.1):
        
        self.channel = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        
        self.last_sent_value = None
        self.last_received_value = None
        
    def send(self, value):
        self.last_sent_value = value        
        self.channel.write(bytes(self.last_sent_value).encode("UTF-8"))
        
    def recv(self, wait_T=7e-2):
        time.sleep(wait_T)
        self.last_received_value = self.channel.readline()
        return self.last_received_value
    
    def handshake(self):
        self.send('[SYNCH]')
        synched = False
        while not synched:
            value = self.recv(5e-1)
            if '[SYNCH]' in value:
                print(value, 'Connection Successful!')
                synched = True
            elif value is not None:
                self.send('[SYNCH]')


# def write_read(x):
#     arduino.write(bytes(x).encode("UTF-8"))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return data
N_vibs = 6
N_lev = 5
vib_order = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]
vib_state = [False] * N_vibs
opts = [str(idx) + ' -> ' + finger for idx,finger in enumerate(vib_order)]
def toggle_finger(val, on):
    if val < N_vibs:
        out = val * N_lev + int(on)
        return out
    else:
        return None

def main():
    print('connecting to device...')
    usb = '/dev/' + [dev.name for dev in serial.tools.list_ports.grep('ttyUSB\d+')][0]

    connection = ArduinoConnection(usb)
    while True:
        try:
            num = input('Toggle vibration: \n' + '\n'.join(opts) + '\n\nchoice: ')
        except :
            pass
        vib_state[num] = not vib_state[num]
        val = toggle_finger(num, vib_state[num])
        print('Sending: {}'.format(val))
        connection.send(val)
        time.sleep(0.05)
        value = connection.recv()
        print(value)

if __name__ == '__main__':
    main()