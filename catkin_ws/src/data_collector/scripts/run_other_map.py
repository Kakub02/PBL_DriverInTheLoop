import carla
import random
import time
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
client.load_world('Town03')