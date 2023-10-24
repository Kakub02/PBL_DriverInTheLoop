import rospy
from std_msgs.msg import Float32, String
from motor_control import set_motor_speed, cleanup
from speed_to_pwm import speed_to_pwm

pwm_range = 100 #wiem drugi raz to deklaruje

def speed_direction_callback(data):
    print(data)
    speed, reverse = data.data.split(',')
    speed_kmph = float(speed)

    max_motor_speed_kmph = 10.0  # trza to dostosować

    pwm_value = speed_to_pwm(speed_kmph, max_motor_speed_kmph, pwm_range)

    set_motor_speed(pwm_value, reverse)

if __name__ == '__main__':
    try:
        rospy.init_node('motor_control', anonymous=True)
        rospy.Subscriber('desired_speed_direction', String, speed_direction_callback)
        rospy.spin()
    finally:
        cleanup()