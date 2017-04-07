/* -------------------------------------------------------------------------- *
 *         Orientation Based Inverse Kinematics : osimExtendedIKDLL.h         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2016-2017 L. Tagliapietra, E. Ceseracciu, M. Reggiani        *
 *                                                                            *
 * Author(s): E. Ceseracciu, L. Tagliapietra (Mar 2016)                       *
 *                                                                            *
 * Contact(s): tagliapietra.work@gmail.com                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at:                                   *
 * http://www.apache.org/licenses/LICENSE-2.0                                 *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef __osimExtendedIK_DLL_h__
#define __osimExtendedIK_DLL_h__


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


#endif // __osimExtendedIK_DLL_h__
