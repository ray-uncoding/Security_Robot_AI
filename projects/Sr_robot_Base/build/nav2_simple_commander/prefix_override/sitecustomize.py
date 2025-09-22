import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nvidia/workspace/Sr_robot_Base/install/nav2_simple_commander'
