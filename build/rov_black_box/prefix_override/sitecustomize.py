import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anastasiia/ros2_uji_projectfinal/install/rov_black_box'
