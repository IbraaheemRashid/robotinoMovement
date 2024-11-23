import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mirashid/robotino_ws/src/robotino_hal/install/robotino_hal'
