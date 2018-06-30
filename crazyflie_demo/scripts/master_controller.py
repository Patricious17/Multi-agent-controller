#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class Controller():
    def __init__(self, joy_topic):

	rospy.loginfo("waiting for joy2master service")
    rospy.wait_for_service('joy2master')
    rospy.loginfo("found joy2master service")
    self._joy2master = rospy.ServiceProxy('joy2master', Empty)
    # subscribe to the joystick at the end to make sure that all required
    # services were found
    self._buttons = None
    rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1:
                    
                if i == 1 and data.buttons[i] == 1:
                    
                if i == 2 and data.buttons[i] == 1:
                    
                if i == 4 and data.buttons[i] == 1:
                    self._joy2master()

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(joy_topic)
    rospy.spin()
