# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/ubuntu/catkin_ws/devel_isolated/zed_wrapper;/home/ubuntu/catkin_ws/devel_isolated/ublox_gps;/home/ubuntu/catkin_ws/devel_isolated/ublox_msgs;/home/ubuntu/catkin_ws/devel_isolated/ublox_serialization;/home/ubuntu/catkin_ws/devel_isolated/ublox;/home/ubuntu/catkin_ws/devel_isolated/simple_navigation_goals;/home/ubuntu/catkin_ws/devel_isolated/sick_ldmrs_tools;/home/ubuntu/catkin_ws/devel_isolated/sick_ldmrs_driver;/home/ubuntu/catkin_ws/devel_isolated/sick_ldmrs_msgs;/home/ubuntu/catkin_ws/devel_isolated/sick_ldmrs_laser;/home/ubuntu/catkin_ws/devel_isolated/sick_ldmrs_description;/home/ubuntu/catkin_ws/devel_isolated/rviz_imu_plugin;/home/ubuntu/catkin_ws/devel_isolated/robot_setup_tf;/home/ubuntu/catkin_ws/devel_isolated/phidgets_imu;/home/ubuntu/catkin_ws/devel_isolated/phidgets_ik;/home/ubuntu/catkin_ws/devel_isolated/phidgets_high_speed_encoder;/home/ubuntu/catkin_ws/devel_isolated/phidgets_drivers;/home/ubuntu/catkin_ws/devel_isolated/phidgets_api;/home/ubuntu/catkin_ws/devel_isolated/line_tracking;/home/ubuntu/catkin_ws/devel_isolated/libphidget21;/home/ubuntu/catkin_ws/devel_isolated/imu_tools;/home/ubuntu/catkin_ws/devel_isolated/imu_filter_madgwick;/home/ubuntu/catkin_ws/devel_isolated/imu_complementary_filter;/home/ubuntu/catkin_ws/devel_isolated/gps;/home/ubuntu/catkin_ws/devel_isolated/charlie_code;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ubuntu/catkin_ws/devel_isolated/rviz_imu_plugin/env.sh')

output_filename = '/home/ubuntu/catkin_ws/build_isolated/rviz_imu_plugin/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
