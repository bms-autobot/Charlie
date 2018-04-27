# CMake generated Testfile for 
# Source directory: /home/ubuntu/catkin_ws/src/phidgets_drivers/phidgets_imu
# Build directory: /home/ubuntu/catkin_ws/build_isolated/phidgets_imu
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_phidgets_imu_roslaunch-check_launch "/home/ubuntu/catkin_ws/build_isolated/phidgets_imu/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/catkin_ws/build_isolated/phidgets_imu/test_results/phidgets_imu/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/ubuntu/catkin_ws/build_isolated/phidgets_imu/test_results/phidgets_imu" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/ubuntu/catkin_ws/build_isolated/phidgets_imu/test_results/phidgets_imu/roslaunch-check_launch.xml' '/home/ubuntu/catkin_ws/src/phidgets_drivers/phidgets_imu/launch' ")
subdirs(gtest)
