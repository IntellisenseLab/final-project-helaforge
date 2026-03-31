import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sudil-minthaka/Documents/final-project-helaforge/install/voice_control_logic'
