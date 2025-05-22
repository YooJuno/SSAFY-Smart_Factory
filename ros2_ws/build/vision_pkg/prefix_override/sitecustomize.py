import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssafy/SSAFY-Smart_Factory/ros2_ws/install/vision_pkg'
