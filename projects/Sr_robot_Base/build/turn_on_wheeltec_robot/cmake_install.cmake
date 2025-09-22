# Install script for directory: /home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/workspace/Sr_robot_Base/install/turn_on_wheeltec_robot")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/turn_on_wheeltec_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_c/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_typesupport_fastrtps_c/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/opt/ros/humble/lib:/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_cpp/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_typesupport_fastrtps_cpp/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_typesupport_introspection_c/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/turn_on_wheeltec_robot/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_typesupport_introspection_cpp/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_python/turn_on_wheeltec_robot/turn_on_wheeltec_robot.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot" TYPE DIRECTORY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/nvidia/workspace/Sr_robot_Base/install/turn_on_wheeltec_robot/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot:/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_fastrtps_c.cpython-310-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot:/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_introspection_c.cpython-310-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot:/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/turn_on_wheeltec_robot/turn_on_wheeltec_robot_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_generator_py/turn_on_wheeltec_robot/libturn_on_wheeltec_robot__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so"
         OLD_RPATH "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libturn_on_wheeltec_robot__rosidl_generator_py.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/msg" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_adapter/turn_on_wheeltec_robot/msg/Position.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/msg" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot/msg/Position.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot" TYPE PROGRAM FILES "/home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot/scripts/cmd_vel_to_ackermann_drive.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot" TYPE EXECUTABLE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/wheeltec_robot_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node"
         OLD_RPATH "/opt/ros/humble/lib:/home/nvidia/workspace/Sr_robot_Base/install/ackermann_msgs/lib:/home/nvidia/workspace/Sr_robot_Base/install/wheeltec_robot_msg/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turn_on_wheeltec_robot/wheeltec_robot_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE DIRECTORY FILES
    "/home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot/launch"
    "/home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/turn_on_wheeltec_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/turn_on_wheeltec_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/environment" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_index/share/ament_index/resource_index/packages/turn_on_wheeltec_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_cppExport.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/turn_on_wheeltec_robot__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport.cmake"
         "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/CMakeFiles/Export/share/turn_on_wheeltec_robot/cmake/export_turn_on_wheeltec_robot__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot/cmake" TYPE FILE FILES
    "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_core/turn_on_wheeltec_robotConfig.cmake"
    "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/ament_cmake_core/turn_on_wheeltec_robotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turn_on_wheeltec_robot" TYPE FILE FILES "/home/nvidia/workspace/Sr_robot_Base/turn_on_wheeltec_robot/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/turn_on_wheeltec_robot__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nvidia/workspace/Sr_robot_Base/build/turn_on_wheeltec_robot/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
