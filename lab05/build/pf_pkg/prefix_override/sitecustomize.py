import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thilevin/sesasr/SESASR/lab05/install/pf_pkg'
