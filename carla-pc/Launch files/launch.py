import subprocess
import threading
import time

master_uri = "192.168.0.101:11311"
rpi_ssh = "jakub@192.168.0.103"
rpi_pass = "2905"

def run_command_in_terminal(command, terminal_title):
    full_command = f'gnome-terminal --title="{terminal_title}" -- bash -c "{command}; exec bash"'
    subprocess.run(full_command, shell=True)

commands_groups = [
    ['roscore'],
    ['cd /home/aczapla/CARLA_0.9.13', './CarlaUE4.sh'],
    ['cd /home/jkawalec', 'python3 import_map.py'],
    ['cd /home/aczapla/catkin_ws/src/ros-bridge/carla_ros_bridge/launch', 'source /opt/ros/noetic/setup.bash', 'source ../../../../devel/setup.bash', 'roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch'],
    ['echo main.py is running...','cd /home/jkawalec/catkin_ws/src/data_collector/scripts', 'source /opt/ros/noetic/setup.bash', 'source ../../../devel/setup.bash', 'rosrun data_collector main.py'],
    ['echo draw_plots.py is running...','cd /home/jkawalec/catkin_ws/src/data_collector/scripts', 'source /opt/ros/noetic/setup.bash', 'source ../../../devel/setup.bash', 'rosrun data_collector draw_plots.py'],
    [f' sshpass -p {rpi_pass} ssh {rpi_ssh}'],
    [f' sshpass -p {rpi_pass} ssh {rpi_ssh}'],
    [f' sshpass -p {rpi_pass} ssh {rpi_ssh}'],
]

def run_commands_in_separate_terminals(commands_groups):
    for i, commands in enumerate(commands_groups, start=1):
        threading.Thread(target=run_command_in_terminal, args=(' ; '.join(commands), f'Terminal {i}')).start()
    
        time.sleep(4)

run_commands_in_separate_terminals(commands_groups)
