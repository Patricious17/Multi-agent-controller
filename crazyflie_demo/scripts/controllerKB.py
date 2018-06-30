#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Requesting services


t : Take-off
b : Land
q : Emergency

CTRL-C to quit

"""

class Controller():
    def __init__(self, use_controller):
        if use_controller:        	
        	# rospy.wait_for_service('update_params')
        	# rospy.loginfo("found update_params service")
        	# self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
        	# rospy.loginfo("waiting for emergency service")
        	# rospy.wait_for_service('emergency')
        	# rospy.loginfo("found emergency service")
        	# self._emergency = rospy.ServiceProxy('emergency', Empty)
        	rospy.loginfo("waiting for land service")
        	rospy.wait_for_service('land')
        	rospy.loginfo("found land service")
        	self._land = rospy.ServiceProxy('land', Empty)
        	rospy.loginfo("waiting for takeoff service")
        	rospy.wait_for_service('takeoff')
        	rospy.loginfo("found takeoff service")
        	self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
        	self._land = None
        	self._takeoff = None

    def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		print "key pressed : ",key
		return key

    def _newKey(self):
		print("New key")
		newkey = self.getKey()
		if newkey == 'b':
			print ("Landing")
			self._land()
		elif newkey == 'e':
			self._emergency()
			print("Emergency")
		elif newkey == 't':
			self._takeoff()
			print("Taking-off")
		else:
			if (newkey == '\x03'):
				self._emergency()
				print("Emergency")

if __name__=="__main__":
	rospy.init_node('crazyflie_demo_controller', anonymous=True)
	settings = termios.tcgetattr(sys.stdin)
	use_controller = rospy.get_param("~use_crazyflie_controller", False)
	controller = Controller(use_controller)
	
	try:
		print msg
		# print vels(speed,turn)
		while(1):
			controller._newKey()

	except:
		print ("ERROR1")

	finally:

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

