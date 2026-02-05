import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nickolas/ros2_ws/src/omni_src/install/omni-pi-server'
