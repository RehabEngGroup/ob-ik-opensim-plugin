/* -------------------------------------------------------------------------- *
 *       Orientation Based Inverse Kinematics : OrientationSensorSet.h        *
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

#ifndef ORIENTATION_SENSOR_SET_H_
#define ORIENTATION_SENSOR_SET_H_

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include "OpenSim/Simulation/Model/OrientationSensor.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Model;
class ScaleSet;
class Body;

class OSIMEXTENDEDIK_API OrientationSensorSet : public ModelComponentSet<OrientationSensor> {
    OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorSet, ModelComponentSet<OrientationSensor>);

private:
    void setNull();
public:
    OrientationSensorSet();
    OrientationSensorSet(Model& aModel, const std::string& aOSensorFileName) SWIG_DECLARE_EXCEPTION;
    OrientationSensorSet(const OrientationSensorSet& aOSensorSet);
    ~OrientationSensorSet(void);

#ifndef SWIG
    OrientationSensorSet& operator=(const OrientationSensorSet &aOSensorSet);
#endif

    void getOSensorNames(Array<std::string>& aOSensorNamesArray);
    void scale(const ScaleSet& aScaleSet);
    /** Add a prefix to sensor names for all orientation sensors in the set**/
    void addNamePrefix(const std::string& prefix);
    OrientationSensor* addOrientationSensor(const std::string& aName, const SimTK::Vec3& aPositionOffset, const SimTK::Vec3& aRotationOffset, OpenSim::Body& aBody);

};  // END of class OrientationSensorSet

} // end of namespace OpenSim

#endif // ORIENTATION_SENSOR_SET_H_
