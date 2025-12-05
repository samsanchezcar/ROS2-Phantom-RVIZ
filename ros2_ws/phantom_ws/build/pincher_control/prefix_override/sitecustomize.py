import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samsanchez/Documents/Robotics/Pincher/ros2_ws/phantom_ws/install/pincher_control'
