#ifndef __OrientationSensorSet_h__
#define __OrientationSensorSet_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  OrientationSensorSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of orientation sensors for inverse kinematics.
 *
 * @authors Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMEXTENDEDIK_API OrientationSensorSet : public ModelComponentSet<OrientationSensor> {
    OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorSet, ModelComponentSet<OrientationSensor>);

private:
    void setNull();
public:
    OrientationSensorSet();
    OrientationSensorSet(Model& aModel, const std::string& aOSensorFileName) SWIG_DECLARE_EXCEPTION;
    OrientationSensorSet(const OrientationSensorSet& aOSensorSet);
    ~OrientationSensorSet(void);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    OrientationSensorSet& operator=(const OrientationSensorSet &aOSensorSet);
#endif

    //--------------------------------------------------------------------------
    // UTILITIES
    //--------------------------------------------------------------------------
    void getOSensorNames(Array<std::string>& aOSensorNamesArray);
    void scale(const ScaleSet& aScaleSet);
    /** Add a prefix to sensor names for all orientation sensors in the set**/
    void addNamePrefix(const std::string& prefix);
    OrientationSensor* addOrientationSensor(const std::string& aName, const SimTK::Vec3& aPositionOffset, const SimTK::Vec3& aRotationOffset, OpenSim::Body& aBody);

//=============================================================================
};  // END of class OrientationSensorSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __OrientationSensorSet_h__
