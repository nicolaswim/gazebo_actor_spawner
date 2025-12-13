import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wim/Documents/gazebo_actor_spawner/install/gazebo_actor_spawner'
