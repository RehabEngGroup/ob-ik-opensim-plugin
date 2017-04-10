/* -------------------------------------------------------------------------- *
 *   Orientation Based Inverse Kinematics : RegisterTypes_osimExtendedIK.h    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2016-2017 L. Tagliapietra, E. Ceseracciu, M. Reggiani        *
 *                                                                            *
 * Author(s): L. Tagliapietra, E. Ceseracciu (Mar 2016)                       *
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

#ifndef REGISTER_TYPES_OSIM_EXTENDED_IK_H_
#define REGISTER_TYPES_OSIM_EXTENDED_IK_H_

#include "osimExtendedIKDLL.h"


extern "C" {

OSIMEXTENDEDIK_API void RegisterTypes_osimExtendedIK();

}

class dllObjectInstantiator
{
public:
        dllObjectInstantiator();
private:
        void registerDllClasses();
};

#endif // REGISTER_TYPES_OSIM_EXTENDED_IK_H_
