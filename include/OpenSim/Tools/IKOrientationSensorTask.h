/* -------------------------------------------------------------------------- *
 *      Orientation Based Inverse Kinematics : IKOrientationSensorTask.h      *
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

#ifndef __IKOrientationSensorTask_h__
#define __IKOrientationSensorTask_h__

#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include "OpenSim/Tools/IKTask.h"

namespace OpenSim {

class OSIMEXTENDEDIK_API IKOrientationSensorTask : public IKTask {
    OpenSim_DECLARE_CONCRETE_OBJECT(IKOrientationSensorTask, IKTask);

public:
    IKOrientationSensorTask();
    IKOrientationSensorTask(const IKOrientationSensorTask &aIKOrientationSensorTask);

#ifndef SWIG
    IKOrientationSensorTask& operator=(const IKOrientationSensorTask &aIKOrientationSensorTask);
#endif


};  // END of class IKOrientationSensorTask

} // end of namespace OpenSim

#endif // __IKOrientationSensorTask_h__
