import rospy
from std_msgs.msg import String

def talk():
	pub = rospy.Publisher('talking_topic', String, queue_size = 10)
	rospy.init_node('publisher', anonymous=True)
	rate = rospy.Rate(1)

	rospy.loginfo("Publisher Node Started")
	while not rospy.is_shutdown():
		msg = "Hello %s" % rospy.get_time()
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talk()
	except	rospy.ROSInterruptException:
		pass
