#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 11/03/20
'''
from __future__ import division, print_function
import rospy
from geometry_msgs.msg import WrenchStamped
from haptic_glove_ros.msg import Vibration
import numpy as np

def logistic(x ,x0, k, L):
    det = 1 + np.exp( -k * (x - x0))
    return L / det


class Optoforce_Vibrator_Bridge():
    opto_finger_order = ['Ring', 'Middle', 'Index', 'Thumb']
    opto_2_glove_pin = [Vibration.pin_ring, Vibration.pin_middle, Vibration.pin_index, Vibration.pin_thumb]
    opto_len = 4
    __opto_min = 10
    __opto_max = 25

    def __init__(self, optoforce_topic_base="optoforce_wrench_", glove_publishing_topic="vibration_state"):
        
        
        self.glovepub = rospy.Publisher(glove_publishing_topic, Vibration, queue_size=1)
        self.optosub = []
        self.__vibropin_state = [0] * 6
        self.magnitude_filter = ([0.0] * 6, [0] * 6)
        for i in range(self.opto_len):
            # lambda_callback = lambda msg: self.callback(i, msg)
            topic = optoforce_topic_base + "{:d}".format(i)
            self.optosub.append(rospy.Subscriber(topic, WrenchStamped, self.callback, callback_args=i, queue_size=1))
            rospy.loginfo(rospy.get_name().split('/')[1] + ": Subscribing to {:s} -> Allegro {:s}".format(topic, self.opto_finger_order[i]))
        rospy.loginfo("Initialization of optoforce to haptic device complete!")

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

    def publish_state(self):
        vib = Vibration()
        vib.levels_per_pin = self.__vibropin_state
        self.glovepub.publish(vib)

    def callback(self, msg, args):
        opto_idx = args
        new_val = self.__wrench2level(self.opto_2_glove_pin[opto_idx], msg)
        if new_val is not self.__vibropin_state[self.opto_2_glove_pin[opto_idx]]:
            self.__vibropin_state[self.opto_2_glove_pin[opto_idx]] = new_val
            self.publish_state()
        # print(msg)

    def on_shutdown(self):
        rospy.loginfo("Shutting down the optoforce->haptic_glove connection...")
        for sub in self.optosub:
            sub.unregister()
        rospy.sleep(.5)
        self.reset_vibration()
        self.glovepub.unregister()
        rospy.loginfo("optoforce->haptic_glove connection closed!")
        rospy.signal_shutdown("End of Node")

    def reset_vibration(self):
        self.__vibropin_state = [0] * 6
        self.publish_state()
