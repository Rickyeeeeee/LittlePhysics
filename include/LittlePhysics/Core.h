#pragma once

#define LP_PLATFORM_WINDOWS
#define LP_BUILD_DLL

#define PI 3.14159265359

#ifdef LP_PLATFORM_WINDOWS
#ifdef LP_BUILD_DLL
	#define LP_API __declspec(dllexport)
#else
	#define LP_API __declspec(dllimport)
#endif // LP_BUILD_DLL

#else
	#define LP_API 
#endif // LP_PLATFORM_WINDOWS
