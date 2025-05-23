import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssafy/Desktop/SSAFY-Smart_Factory/pjt_pkg/install/yolo_pkg'
