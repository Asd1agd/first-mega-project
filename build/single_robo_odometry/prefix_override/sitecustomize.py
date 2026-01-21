import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asd/temp_git_repo/first-mega-project/install/single_robo_odometry'
