import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/nimsika/New Volume/semester-04/robotic-and-automation/final-project/final-project-helaforge/install/voice_control_logic'
