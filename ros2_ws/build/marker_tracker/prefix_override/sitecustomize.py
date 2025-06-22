import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chakrapani/drone_files/drone_precise_delivery/ros2_ws/install/marker_tracker'
