/* -------------------------------------------------------------------------- *
 *      Orientation Based Inverse Kinematics: OrientationSensorFrame.cpp      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Common/OrientationSensorFrame.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationSensorFrame::OrientationSensorFrame() :
    _numOrientationSensors(0),
    _frameNumber(-1),
    _units()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Constructor taking all the header information
 *
 * @param aNumOrientationSensors the number of oSensors in the frame
 * @param aFrameNumber the frame number
 * @param aTime the time of the frame
 * @param aUnits the units of the orientation sensor's quaternion
 */
OrientationSensorFrame::OrientationSensorFrame(int aNumOrientationSensors, int aFrameNumber, double aTime, Units& aUnits) :
    _numOrientationSensors(aNumOrientationSensors),
    _frameNumber(aFrameNumber),
    _frameTime(aTime),
    _units(aUnits)
{
  setNull();
}

/**
 * Copy constructor.
 */
OrientationSensorFrame::OrientationSensorFrame(const OrientationSensorFrame& aFrame) :
   Object(aFrame)
{
    setNull();

    _numOrientationSensors = aFrame._numOrientationSensors;
    _frameNumber = aFrame._frameNumber;
    _frameTime = aFrame._frameTime;
    _units = aFrame._units;
    _orientationSensors = aFrame._orientationSensors;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationSensorFrame::~OrientationSensorFrame()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this OrientationSensorFrame to their null values.
 */
void OrientationSensorFrame::setNull()
{
    setAuthors("Luca Tagliapietra, Elena Ceseracciu");
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a oSensor to the frame
 *
 * @param aQuaternion the quaternion of the orientation sensor
 */
void OrientationSensorFrame::addOrientationSensor(const SimTK::Quaternion& aQuaternion)
{
    _orientationSensors.push_back(aQuaternion);
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the quaternions of all orientation sensors in the frame
 * TODO: check if make sense
 *
 * @param aScaleFactor the scale factor
 */
void OrientationSensorFrame::scale(double aScaleFactor)
{
    for (int i = 0; i < _numOrientationSensors; i++)
    {
        _orientationSensors[i] *= aScaleFactor;
    }
}
