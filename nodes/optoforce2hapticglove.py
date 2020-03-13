#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 11/03/20
'''
from __future__ import division, print_function
from optoforce2glove import Optoforce_Vibrator_Bridge
import rospy




if __name__ == "__main__":
    rospy.init_node("tactile_haptic_controller")
    opto2glove = Optoforce_Vibrator_Bridge()
    rospy.on_shutdown(opto2glove.on_shutdown)
    rospy.spin()
    # rate = rospy.Rate(30)
    # while not rospy.is_shutdown():
    #     # opto2glove.publish_state()
    #     rate.sleep()

    
