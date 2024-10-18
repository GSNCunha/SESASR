import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lincunha/SESASR/lab01/src/lab01_pkg/install/lab01_pkg'
