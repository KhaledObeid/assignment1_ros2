import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/khaledob/bookros2_ws/install/br2_fsm_bumpgo_py'
