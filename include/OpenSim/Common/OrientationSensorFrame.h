/* -------------------------------------------------------------------------- *
 *      Orientation Based Inverse Kinematics : OrientationSensorFrame.h       *
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

#ifndef __OrientationSensorFrame_h__
#define __OrientationSensorFrame_h__

#include <iostream>
#include <string>
#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include "OpenSim/Common/Object.h"
#include "OpenSim/Common/ArrayPtrs.h"
#include "OpenSim/Common/Units.h"

namespace OpenSim {

class OSIMEXTENDEDIK_API OrientationSensorFrame : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorFrame, Object);

private:
    int _numOrientationSensors;
    int _frameNumber;
    double _frameTime;
    Units _units;
    SimTK::Array_<SimTK::Quaternion> _orientationSensors;

public:
    OrientationSensorFrame();
    OrientationSensorFrame(int aNumOrientationSensors, int aFrameNumber, double aTime, Units& aUnits);
    OrientationSensorFrame(const OrientationSensorFrame& aFrame);
    virtual ~OrientationSensorFrame();

    void addOrientationSensor(const SimTK::Quaternion& aQuaternion);
    SimTK::Quaternion getOrientationSensor(int aIndex) const { return _orientationSensors[aIndex]; }
    SimTK::Quaternion& updOrientationSensor(int aIndex) { return _orientationSensors[aIndex]; }
    int getFrameNumber() const { return _frameNumber; }
    void setFrameNumber(int aNumber) { _frameNumber = aNumber; }
    double getFrameTime() const { return _frameTime; }
    void scale(double aScaleFactor);

    const SimTK::Array_<SimTK::Quaternion>& getOrientationSensors() const { return _orientationSensors;}

private:
    void setNull();

};  // END of class OrientationSensorFrame

} // end of namespace OpenSim

#endif // __OrientationSensorFrame_h__


