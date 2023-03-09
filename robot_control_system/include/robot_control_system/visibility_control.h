#ifndef ROBOT_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_
#define ROBOT_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOT_CONTROL_SYSTEM_EXPORT __attribute__((dllexport))
#define ROBOT_CONTROL_SYSTEM_IMPORT __attribute__((dllimport))
#else
#define ROBOT_CONTROL_SYSTEM_EXPORT __declspec(dllexport)
#define ROBOT_CONTROL_SYSTEM_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOT_CONTROL_SYSTEM_BUILDING_DLL
#define ROBOT_CONTROL_SYSTEM_PUBLIC ROBOT_CONTROL_SYSTEM_EXPORT
#else
#define ROBOT_CONTROL_SYSTEM_PUBLIC ROBOT_CONTROL_SYSTEM_IMPORT
#endif
#define ROBOT_CONTROL_SYSTEM_PUBLIC_TYPE ROBOT_CONTROL_SYSTEM_PUBLIC
#define ROBOT_CONTROL_SYSTEM_LOCAL
#else
#define ROBOT_CONTROL_SYSTEM_EXPORT __attribute__((visibility("default")))
#define ROBOT_CONTROL_SYSTEM_IMPORT
#if __GNUC__ >= 4
#define ROBOT_CONTROL_SYSTEM_PUBLIC __attribute__((visibility("default")))
#define ROBOT_CONTROL_SYSTEM_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOT_CONTROL_SYSTEM_PUBLIC
#define ROBOT_CONTROL_SYSTEM_LOCAL
#endif
#define ROBOT_CONTROL_SYSTEM_PUBLIC_TYPE
#endif

#endif  // ROBOT_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_
