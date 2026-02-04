import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/turtle_ws/src/cps_pid_turtle/install/cps_pid_turtle'
