/* -------------------------------------------------------------------------- *
 *        Orientation Based Inverse Kinematics : IKExtendedTaskSet.h          *
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

#ifndef __IKExtendedTaskSet_h__
#define __IKExtendedTaskSet_h__

#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include <OpenSim/Common/Set.h>
#include "OpenSim/Tools/IKTask.h"
#include "OpenSim/Tools/IKTaskSet.h"
#include "OpenSim/Tools/IKMarkerTask.h"
#include "OpenSim/Tools/IKOrientationSensorTask.h"
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/OrientationSensorsReference.h>

namespace OpenSim {

class OSIMEXTENDEDIK_API IKExtendedTaskSet : public OpenSim::IKTaskSet {
OpenSim_DECLARE_CONCRETE_OBJECT(IKExtendedTaskSet, IKTaskSet);

public:
    IKExtendedTaskSet() {}
    IKExtendedTaskSet(const IKExtendedTaskSet &aIKExtendedTaskSet) : IKTaskSet(aIKExtendedTaskSet) { }
    IKExtendedTaskSet(const std::string &aFileName) : IKTaskSet(aFileName) { }

    void createOrientationSensorsWeightSet(Set<OrientationSensorWeight>& aWeights){
        for (int i = 0; i < getSize(); i++){
            if (IKOrientationSensorTask *nextTask = dynamic_cast<IKOrientationSensorTask *>(&get(i))){
                if (nextTask->getApply()){
                    aWeights.cloneAndAppend(*(new OrientationSensorWeight(nextTask->getName(), nextTask->getWeight())));
                }
            }
        }
    };
#ifndef SWIG
    IKExtendedTaskSet& operator=(const IKExtendedTaskSet &aIKExtendedTaskSet) { IKTaskSet::operator=(aIKExtendedTaskSet); return *this; }
#endif
//=============================================================================
};  // END of class IKExtendedTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKExtendedTaskSet_h__
