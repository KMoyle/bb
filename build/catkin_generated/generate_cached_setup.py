# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
<<<<<<< HEAD
<<<<<<< refs/remotes/origin/master
    for workspace in '/home/nanorobot/baebot_ws/devel;/home/nanorobot/catkin_ws/devel;/opt/ros/melodic'.split(';'):
=======
    for workspace in '/home/kyle/gazebo_ws/devel;/home/kyle/baebot_ws/devel;/home/kyle/cartographer_ws/devel;/home/kyle/catkin_ws/devel;/opt/ros/melodic'.split(';'):
>>>>>>> fix
=======
    for workspace in '/home/kyle/gazebo_ws/devel;/home/kyle/baebot_ws/devel;/home/kyle/cartographer_ws/devel;/home/kyle/catkin_ws/devel;/opt/ros/melodic'.split(';'):
>>>>>>> devel
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

<<<<<<< refs/remotes/origin/master
code = generate_environment_script('/home/nanorobot/baebot_ws/devel/env.sh')

output_filename = '/home/nanorobot/baebot_ws/build/catkin_generated/setup_cached.sh'
=======
code = generate_environment_script('/home/kyle/baebot_ws/devel/env.sh')

output_filename = '/home/kyle/baebot_ws/build/catkin_generated/setup_cached.sh'
>>>>>>> fix
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
