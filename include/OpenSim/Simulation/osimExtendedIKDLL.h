#ifndef OSIMEXTENDEDIK_DLL_H_
#define OSIMEXTENDEDIK_DLL_H_


// UNIX PLATFORM
#ifndef WIN32

#define OSIMEXTENDEDIK_API

// WINDOWS PLATFORM
#else

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#ifdef OSIMEXTENDEDIK_EXPORTS
#define OSIMEXTENDEDIK_API __declspec(dllexport)
#else
#define OSIMEXTENDEDIK_API __declspec(dllimport)
#endif

#endif // PLATFORM


#endif // OSIMEXTENDEDIK_DLL_H_
