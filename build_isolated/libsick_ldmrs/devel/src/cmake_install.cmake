# Install script for directory: /home/ubuntu/catkin_ws/src/libsick_ldmrs/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "bin")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin" TYPE EXECUTABLE FILES "/home/ubuntu/catkin_ws/build_isolated/libsick_ldmrs/devel/src/LDMRS_Example")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/bin/LDMRS_Example")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  foreach(file
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0.1.0"
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0"
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0.1.0;/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0;/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib" TYPE SHARED_LIBRARY FILES
    "/home/ubuntu/catkin_ws/build_isolated/libsick_ldmrs/devel/src/libsick_ldmrs.so.0.1.0"
    "/home/ubuntu/catkin_ws/build_isolated/libsick_ldmrs/devel/src/libsick_ldmrs.so.0"
    "/home/ubuntu/catkin_ws/build_isolated/libsick_ldmrs/devel/src/libsick_ldmrs.so"
    )
  foreach(file
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0.1.0"
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so.0"
      "$ENV{DESTDIR}/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/lib/libsick_ldmrs.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/include/sick_ldmrs/.")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/catkin_ws/devel_isolated/libsick_ldmrs/include/sick_ldmrs" TYPE DIRECTORY FILES "/home/ubuntu/catkin_ws/src/libsick_ldmrs/src/." FILES_MATCHING REGEX "/[^/]*\\.hpp$")
endif()

