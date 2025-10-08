import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bubu/week6_ws/src/control_rbkairos/install/control_rbkairos'
