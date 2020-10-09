#ifndef CRANE_PLUS_IGN__VISIBILITY_CONTROL_H_
#define CRANE_PLUS_IGN__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CRANE_PLUS_IGN_EXPORT __attribute__ ((dllexport))
    #define CRANE_PLUS_IGN_IMPORT __attribute__ ((dllimport))
  #else
    #define CRANE_PLUS_IGN_EXPORT __declspec(dllexport)
    #define CRANE_PLUS_IGN_IMPORT __declspec(dllimport)
  #endif
  #ifdef CRANE_PLUS_IGN_BUILDING_DLL
    #define CRANE_PLUS_IGN_PUBLIC CRANE_PLUS_IGN_EXPORT
  #else
    #define CRANE_PLUS_IGN_PUBLIC CRANE_PLUS_IGN_IMPORT
  #endif
  #define CRANE_PLUS_IGN_PUBLIC_TYPE CRANE_PLUS_IGN_PUBLIC
  #define CRANE_PLUS_IGN_LOCAL
#else
  #define CRANE_PLUS_IGN_EXPORT __attribute__ ((visibility("default")))
  #define CRANE_PLUS_IGN_IMPORT
  #if __GNUC__ >= 4
    #define CRANE_PLUS_IGN_PUBLIC __attribute__ ((visibility("default")))
    #define CRANE_PLUS_IGN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CRANE_PLUS_IGN_PUBLIC
    #define CRANE_PLUS_IGN_LOCAL
  #endif
  #define CRANE_PLUS_IGN_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CRANE_PLUS_IGN__VISIBILITY_CONTROL_H_