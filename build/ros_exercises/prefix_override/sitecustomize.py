import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/racecar/racecar_ws/src/lab1c/install/ros_exercises'
