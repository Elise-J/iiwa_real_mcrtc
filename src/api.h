#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define IiwaFsmController_DLLIMPORT __declspec(dllimport)
#  define IiwaFsmController_DLLEXPORT __declspec(dllexport)
#  define IiwaFsmController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define IiwaFsmController_DLLIMPORT __attribute__((visibility("default")))
#    define IiwaFsmController_DLLEXPORT __attribute__((visibility("default")))
#    define IiwaFsmController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define IiwaFsmController_DLLIMPORT
#    define IiwaFsmController_DLLEXPORT
#    define IiwaFsmController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef IiwaFsmController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define IiwaFsmController_DLLAPI
#  define IiwaFsmController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef IiwaFsmController_EXPORTS
#    define IiwaFsmController_DLLAPI IiwaFsmController_DLLEXPORT
#  else
#    define IiwaFsmController_DLLAPI IiwaFsmController_DLLIMPORT
#  endif // IiwaFsmController_EXPORTS
#  define IiwaFsmController_LOCAL IiwaFsmController_DLLLOCAL
#endif // IiwaFsmController_STATIC