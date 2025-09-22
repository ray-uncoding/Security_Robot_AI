// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_turn_on_wheeltec_robot __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_turn_on_wheeltec_robot __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_turn_on_wheeltec_robot __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_turn_on_wheeltec_robot __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_turn_on_wheeltec_robot
    #define ROSIDL_GENERATOR_CPP_PUBLIC_turn_on_wheeltec_robot ROSIDL_GENERATOR_CPP_EXPORT_turn_on_wheeltec_robot
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_turn_on_wheeltec_robot ROSIDL_GENERATOR_CPP_IMPORT_turn_on_wheeltec_robot
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_turn_on_wheeltec_robot __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_turn_on_wheeltec_robot
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_turn_on_wheeltec_robot __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_turn_on_wheeltec_robot
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
