from __future__ import  print_function, division, absolute_import

import serial
import serial.threaded
import serial.tools.list_ports

import traceback
import time

BAUD_RATE = 57600



class ArduinoComm(serial.threaded.LineReader):
    
    
    # TERMINATOR = b'\n'
    
    def connection_made(self, transport):
        super(ArduinoComm, self).connection_made(transport)
        print('Port Opened!')
        # self.write_line('[SYNCH]')
        # synch_response = self.read_line()
        # print(synch_response)
        
    def handle_line(self, line):
        print('line received: {}\n'.format(line))
        # return super(ArduinoComm, self).handle_line(line)
    
    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        print('port closed\n')
        return super(ArduinoComm, self).connection_lost(exc)
        
    


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
    ser = serial.Serial(port=usb, baudrate=BAUD_RATE, timeout=5e-1)
    
    with serial.threaded.ReaderThread(ser, ArduinoComm) as protocol:
        while ser.isOpen():
            try:
                num = input('Toggle vibration: \n' + '\n'.join(opts) + '\n\nchoice: ')
            except :
                pass
            vib_state[num] = not vib_state[num]
            val = toggle_finger(num, vib_state[num])
            print('Sending: {}'.format(val))
            # connection.send(val)
            protocol.write_line(str(val))
            time.sleep(0.05)
            

if __name__ == '__main__':
    main()