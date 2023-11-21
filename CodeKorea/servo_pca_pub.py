#!/usr/bin/env python3

import rospy
from servo_control.msg import ServoPosition
from adafruit_servokit import ServoKit

def servo_publisher():
    rospy.init_node('servo_publisher', anonymous=True)
    servo_pub = rospy.Publisher('/servo_position', ServoPosition, queue_size=10)

    #kit = ServoKit(channels=16, address=0x40) 

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            servo1_position = float(input("서보 1 (0~180) : "))
            servo2_position = float(input("서보 2 (0~180) : "))

            if 0 <= servo1_position <= 180 and 0 <= servo2_position <= 180:

                servo_msg = ServoPosition()
                servo_msg.position1 = servo1_position  # 서보 1 위치를 사용
                servo_msg.position2 = servo2_position  # 서보 1 위치를 사용
                
                servo_pub.publish(servo_msg)

            else:
                rospy.logwarn("범위 벗어남")    
        except ValueError:
            rospy.logwarn("재입력")   

        rate.sleep()

if __name__ == '__main__':
    try:
        servo_publisher()
    except rospy.ROSInterruptException:
        pass