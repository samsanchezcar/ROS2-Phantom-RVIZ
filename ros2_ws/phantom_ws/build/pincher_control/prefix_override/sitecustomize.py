import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samsanchez/Documents/Robotics/KIT_Phantom_X_Pincher_ROS2/phantom_ws/install/pincher_control'
