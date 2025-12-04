import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david/Escritorio/UPV/g05_prii3/g05_prii3_WS/install/sprint4_eurobot'
