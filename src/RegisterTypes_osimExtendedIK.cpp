/* -------------------------------------------------------------------------- *
 *   Orientation Based Inverse Kinematics: RegisterTypes_osimExtendedIK.cpp   *
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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "OpenSim/Simulation/RegisterTypes_osimExtendedIK.h"

#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"
#include "OpenSim/Tools/IKOrientationSensorTask.h"
#include "OpenSim/Tools/IKExtendedTaskSet.h"

using namespace OpenSim;
using namespace std;

static dllObjectInstantiator instantiator;

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Plugin library.
 */
OSIMEXTENDEDIK_API void RegisterTypes_osimExtendedIK()
{
    Object::RegisterType(OrientationSensor());
    OpenSim::Object::registerType(OpenSim::IKExtendedTaskSet());
    OpenSim::Object::registerType(OpenSim::IKOrientationSensorTask());
    Object::RegisterType(InverseKinematicsExtendedTool());
}

dllObjectInstantiator::dllObjectInstantiator()
{
        registerDllClasses();
}

void dllObjectInstantiator::registerDllClasses()
{
        RegisterTypes_osimExtendedIK();
}
