import rospy
from std_msgs.msg import String, Float32, Bool

def listener():
	rospy.init_node("data_receiver", anonymous=True)

	rospy.Subscriber('/our_msg/speed', Float32, callback_speed)
	rospy.Subscriber('/our_msg/roll', Float32, callback_roll)
	rospy.Subscriber('/our_msg/pitch', Float32, callback_pitch)
	rospy.Subscriber('/our_msg/reverse', Bool, callback_reverse)
	
	loop = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		loop.sleep()

def callback_speed(msg):
	speed = float(msg.data)
	rospy.loginfo("Speed: %f", speed)

def callback_roll(msg):
	roll = float(msg.data)
	rospy.loginfo("Roll: %f", roll)

def callback_pitch(msg):
	pitch = float(msg.data)
	rospy.loginfo("Pitch: %f", pitch)

def callback_reverse(msg):
	reverse = bool(msg.data)
	rospy.loginfo("Reverse:  %r", reverse)	

def callback(msg):
	rospy.loginfo("Working")

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

