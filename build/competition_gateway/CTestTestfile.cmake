# CMake generated Testfile for 
# Source directory: /home/husky/TR/src/competition_gateway
# Build directory: /home/husky/TR/build/competition_gateway
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_serial_protocol "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/husky/TR/build/competition_gateway/test_results/competition_gateway/test_serial_protocol.gtest.xml" "--package-name" "competition_gateway" "--output-file" "/home/husky/TR/build/competition_gateway/ament_cmake_gtest/test_serial_protocol.txt" "--command" "/home/husky/TR/build/competition_gateway/test_serial_protocol" "--gtest_output=xml:/home/husky/TR/build/competition_gateway/test_results/competition_gateway/test_serial_protocol.gtest.xml")
set_tests_properties(test_serial_protocol PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/husky/TR/build/competition_gateway/test_serial_protocol" TIMEOUT "60" WORKING_DIRECTORY "/home/husky/TR/build/competition_gateway" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/husky/TR/src/competition_gateway/CMakeLists.txt;32;ament_add_gtest;/home/husky/TR/src/competition_gateway/CMakeLists.txt;0;")
subdirs("gtest")
