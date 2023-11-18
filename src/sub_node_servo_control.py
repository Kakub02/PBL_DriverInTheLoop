import rospy
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

GPIO.setmode(GPIO.BCM)

class Servo_Control():

    def __init__(self):
        print("Servo_Control initialized")
        self.pca9685 = ServoKit(channels=16, addres=0x70) #w terminalu zeby sprawdzic : i2detect -y 1
    #     self.kit = ServoKit(channels=16)
        # servoPin = 312
        # SERVO_MAX_DUTY = 12 # 180 degree 100%pwm
        # SERVO_MIN_DUTY= 2 # 0 degree 0% pwm

        # GPIO.setmode(GPIO.BOARD)        
        # GPIO.setup(servoPin, GPIO.OUT)  

        # servo = GPIO.PWM(servoPin, 50)  
        # servo.start(0)
        
    def communication(self):
        rospy.Subscriber('/our_msg/roll', Float32, self.callback_roll)
        rospy.Subscriber('/our_msg/pitch', Float32, self.callback_pitch)


    def callback_roll(self, msg):
        # tutaj wysyłamy info o roll do servo
        roll = float(msg.data)
        self.pca9685.servo[0].angle = roll


    def callback_pitch(self, msg):
        # tutaj wysyłamy info o pitch do servo
        pitch = float(msg.data)
        self.pca9685.servo[1].angle = pitch


    def run(self):
        self.communication()

        loop = rospy.Rate(50.0) # frequency in Hz

        while not rospy.is_shutdown(): 
            loop.sleep()
            GPIO.cleanup()
   
    # def update_servo_roll(self,roll):
    #     duty_cycle = 2 + (roll / 18)
    #     self.servo.ChangeDutyCycle(duty_cycle)
    #     rospy.sleep(0.5)


if __name__ == "__main__":
    #while rosrunning 
    rospy.init_node("Servo_node", anonymous=True)
    servo_controller = Servo_Control()
    servo_controller.run()