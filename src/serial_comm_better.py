from __future__ import  print_function, division, absolute_import

import serial
import serial.threaded
import serial.tools.list_ports

import traceback
import time
import numpy as np
from copy import deepcopy

import rospy
from geometry_msgs.msg import WrenchStamped
from haptic_glove_ros.msg import Vibration


BAUD_RATE = 57600



class ArduinoComm(serial.threaded.LineReader):
    
    VIB_ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]
    # TERMINATOR = b'\n'
    def connection_made(self, transport):
        super(ArduinoComm, self).connection_made(transport)
        # print('Port Opened!')
        self.state = {finger: 0 for finger in GloveConnection.VIB_ORDER}
        # self.write_line('[SYNCH]')
        # synch_response = self.read_line()
        # print(synch_response)
        
    def handle_line(self, line):
        # print('line received: {}\n'.format(line))
        actuators = set(deepcopy(self.VIB_ORDER))
        if line.startswith('[STATUS] '):
            try:
                states = line.split('{')[1].split('}')[0].split(',')[:-1]
                for state in states:
                    finger, lev = state.split(': ')
                    finger = str(finger).strip()
                    actuators = actuators.difference([finger])
                    self.state[finger] = int(lev)
                for finger in actuators:
                    self.state[finger] = 0
            except Exception as e:
                traceback.print_exc(e)
                pass
        elif line.startswith('[ERROR] '):
            print(line)
            # print(self.state)
            # print('current state: {}\n'.format(' '.join(state)))
    
    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        # print('port closed\n')
        return super(ArduinoComm, self).connection_lost(exc)
        
    
class GloveConnection():
    N_VIBS = 6
    N_LEVS = 5
    VIB_ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]
    
    def __init__(self, baud_rate, dev_name=None):
        self.vib_state = [0] * self.N_VIBS
        self.baud_rate = baud_rate
        self.dev_name = next(serial.tools.list_ports.grep('ttyUSB\d+')).device if dev_name is None else dev_name 
        self.connection = serial.Serial(port=self.dev_name, baudrate=baud_rate, timeout=5e-2, write_timeout=5e-2)
        self.thread = serial.threaded.ReaderThread(self.connection, ArduinoComm)
        
        # self.thread.start()
        # self.protocol = self.thread.protocol
        # self.protocol
        time.sleep(2)
        
    def set_vib(self, vib_id, level):
        assert self.connection.isOpen(), 'The connection has been closed'
        self.vib_state[vib_id] = level 
        msg = self.pinlvl2idx(vib_id, level)
        try:
            self.protocol.write_line('[MSG] ' + str(msg))
        except serial.SerialTimeoutException as exc:
            print(exc)
            pass
            # raise exc
        return msg
    
    def pinlvl2idx(self, id, level):    
        assert id < self.N_VIBS, 'max pin id should be {}'.format(self.N_VIBS)
        assert level < self.N_LEVS, 'max level id should be {}'.format(self.N_LEVS)
        
        return id * self.N_LEVS + int(level)
    
    def __getitem__(self, id):
        assert id < self.N_VIBS, 'max pin id should be {}'.format(self.N_VIBS)
        return self.vib_state[id]
    
# opts = [str(idx) + ' -> ' + finger for idx,finger in enumerate(GloveConnection.VIB_ORDER)]
class Glove_Node():
    
    opto_finger_order = ['Index', 'Middle', 'Ring', 'Thumb']
    opto_2_glove_pin = [Vibration.pin_ring, Vibration.pin_middle, Vibration.pin_index, Vibration.pin_thumb]
    opto_len = 4
    __opto_min = 10
    __opto_max = 25
    
    def __init__(self):
        
        self.glove_connection = GloveConnection(BAUD_RATE)
        rospy.init_node('glove_connection')
        rospy.loginfo('Connecting to the haptic glove...')
        self.magnitude_filter = ([0.0] * 6, [0] * 6)
        self.optoforce_subscriber = {finger: \
            rospy.Subscriber('optoforce_wrench_{:d}'.format(idx), WrenchStamped, self.callback, idx) \
                for idx, finger in enumerate(self.opto_finger_order)}
        
        
    def __wrench2level(self, pin, msg):

        tactile_force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        magn = np.linalg.norm(tactile_force, ord=2)
        self.magnitude_filter[1][pin] += 1
        n = self.magnitude_filter[1][pin]
        self.magnitude_filter[0][pin] = (self.magnitude_filter[0][pin] + (n - 1) * magn) / n

        # y = int(np.floor(logistic(magn, 15, 0.3, 4.5))) # min perceived value 10. -  max perceived value 20.
        y = int(np.floor(logistic(self.magnitude_filter[0][pin], 20, 0.1, 4.5))) # between 10- and 50-
        rospy.logdebug(self.opto_finger_order[pin] + " - magnitude {:.3f} -> level {:d}".format(self.magnitude_filter[0][pin], y))
        
        return y
    
    def callback(self, msg, args):
        opto_idx = args
        opto_finger = self.opto_finger_order[opto_idx]
        vib_idx = self.glove_connection.VIB_ORDER.index(opto_finger)
        new_val = self.__wrench2level(vib_idx, msg)
        if new_val is not self.glove_connection.vib_state[vib_idx]:
            self.glove_connection.vib_state[vib_idx] = new_val
            # self.publish_state()
            
    def send_state(self):
        for idx, lev in enumerate(self.glove_connection.vib_state):
            self.glove_connection.set_vib(idx, lev)
            time.sleep(5e-2)
            
    def run(self):
        with self.glove_connection.thread as self.glove_connection.protocol:
            while self.glove_connection.connection.isOpen():
                self.send_state()
                print(self.glove_connection.vib_state)
                time.sleep(1e-1)
        
        
def logistic(x ,x0, k, L):
    det = 1 + np.exp( -k * (x - x0))
    return L / det 

def main():
    glove = GloveConnection(BAUD_RATE)
    with glove.thread as glove.protocol:
        for num in range(glove.N_VIBS):
            lev = int(not glove[num] > 0)
            print(glove.VIB_ORDER[num], lev)
            val = glove.set_vib(num, lev)
            
            time.sleep(.05)
            time.sleep(.5)
            # lev = int(not glove[num] > 0)
            # val = glove.set_vib(num, lev)
            
        for num in range(glove.N_VIBS):
            lev = int(not glove[num] > 0)
            print(glove.VIB_ORDER[num], lev)
            val = glove.set_vib(num, lev)
            
            time.sleep(.05)
            time.sleep(.5)
            # lev = int(not glove[num] > 0)
            # val = glove.set_vib(num, lev)
        # while True:
        #     time.sleep(0.05)
            
            
def node_main():
    
    node = Glove_Node()
    
    while not rospy.is_shutdown():
        node.run()
        node.glove_connection = GloveConnection(BAUD_RATE)
        time.sleep(1)
            

if __name__ == '__main__':
    # main()
    node_main()