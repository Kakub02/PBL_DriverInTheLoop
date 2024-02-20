import os
import subprocess

master_uri = "192.168.0.101:11311"

def run_commands(commands):
    for command in commands:
        subprocess.run(command, shell=True)

os.environ['ROS_MASTER_URI'] = f'http://{master_uri}'

commands = [
    'cd /home/jakub/catkin_ws/src/dil_controller/scripts',
    'rosrun dil_controller sub_servo.py'
]

run_commands(commands)
