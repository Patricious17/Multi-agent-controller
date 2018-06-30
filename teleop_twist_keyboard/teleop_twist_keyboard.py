#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#
# import rospy
# from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
#

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

######################################
#############----COPIED----###########
######################################
#
class Controller():
    def __init__(self, use_controller, joy_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
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

        # # subscribe to the joystick at the end to make sure that all required
        # # services were found
        # self._buttons = None
        # rospy.Subscriber(joy_topic, Joy, self._joyChanged)
#



# moveBindings = {
# 		'i':(1,0,0,0),
# 		'o':(1,0,0,-1),
# 		'j':(0,0,0,1),
# 		'l':(0,0,0,-1),
# 		'u':(1,0,0,1),
# 		',':(-1,0,0,0),
# 		'.':(-1,0,0,1),
# 		'm':(-1,0,0,-1),
# 		'O':(1,-1,0,0),
# 		'I':(1,0,0,0),
# 		'J':(0,1,0,0),
# 		'L':(0,-1,0,0),
# 		'U':(1,1,0,0),
# 		'<':(-1,0,0,0),
# 		'>':(-1,-1,0,0),
# 		'M':(-1,1,0,0),
# 		't':(0,0,1,0),
# 		'b':(0,0,-1,0),
# 	       }

# speedBindings={
# 		'q':(1.1,1.1),
# 		'z':(.9,.9),
# 		'w':(1.1,1),
# 		'x':(.9,1),
# 		'e':(1,1.1),
# 		'c':(1,.9),
# 	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key




# def vels(speed,turn):
	# return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	# pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	# speed = rospy.get_param("~speed", 0.5)
	# turn = rospy.get_param("~turn", 1.0)
	# x = 0
	# y = 0
	# z = 0
	# th = 0
	# status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key is 'b':
				self._land()
			elif key is 'q':
				self._emergency()
			elif key is 't':
				self._takeoff()


				# print vels(speed,turn)
				# if (status == 14):
				# 	print msg
				# status = (status + 1) % 15
			else:
				# x = 0
				# y = 0
				# z = 0
				# th = 0
				if (key == '\x03'):
					break
			# if key in moveBindings.keys():
			# 	x = moveBindings[key][0]
			# 	y = moveBindings[key][1]
			# 	z = moveBindings[key][2]
			# 	th = moveBindings[key][3]
			# elif key in speedBindings.keys():
			# 	speed = speed * speedBindings[key][0]
			# 	turn = turn * speedBindings[key][1]

			# 	print vels(speed,turn)
			# 	if (status == 14):
			# 		print msg
			# 	status = (status + 1) % 15
			# else:
			# 	x = 0
			# 	y = 0
			# 	z = 0
			# 	th = 0
			# 	if (key == '\x03'):
			# 		break

			# twist = Twist()
			# twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			# pub.publish(twist)

	except:
		print e

	finally:
		# twist = Twist()
		# twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


###########___COPIED___##########
#
if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
 #