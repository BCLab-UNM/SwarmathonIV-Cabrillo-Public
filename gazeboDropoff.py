#!/usr/bin/env python
import rospy  
import roslib
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import GetModelState

/gazebo/model_states

def setObject():

	pub = rospy.Publisher('setObjectTopic', ModelState, queue_size=10)
	rospy.init_node('setObject', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while (True): #not rospy.isshutdown()
		print 'test'
		msg = ModelState()
		msg.pose.position.x = 0.0 
		msg.pose.position.y = 0.0 
		msg.pose.position.z = 10.0 
		msg.model_name = "box"
		pub.publish(msg)
		rate.sleep()
		print  "here"


if __name__ == '__main__':
	try:
		 setObject()
	except rospy.ROSInterruptException:
		 pass
		 
		 #model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
