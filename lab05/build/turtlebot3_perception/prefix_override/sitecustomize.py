import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/s2c2/Desktop/SESASR/lab05/install/turtlebot3_perception'
