#ifndef __OrientationSensorData_h__
#define __OrientationSensorData_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  OrientationSensorData.h                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include <iostream>
#include <string>
#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include "OpenSim/Common/Object.h"
#include "OpenSim/Common/Storage.h"
#include "OpenSim/Common/ArrayPtrs.h"
#include "OpenSim/Common/OrientationSensorFrame.h"
#include "OpenSim/Common/Units.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a sequence of orientation sensor frames from a MOT/STO file.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMEXTENDEDIK_API OrientationSensorData : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorData, Object);

//=============================================================================
// DATA
//=============================================================================
private:
    int _numFrames;
    int _numOrientationSensors;
    int _firstFrameNumber;
    double _dataRate;
    double _cameraRate; // TODO check if these info are useless and can be removed
    double _originalDataRate;
    int _originalStartFrame;
    int _originalNumFrames;
    std::string _fileName;
    Units _units;
    Array<std::string> _orientationSensorNames;
    ArrayPtrs<OrientationSensorFrame> _frames;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    OrientationSensorData();
    explicit OrientationSensorData(const std::string& aFileName) SWIG_DECLARE_EXCEPTION;
    virtual ~OrientationSensorData();

    void findFrameRange(double aStartTime, double aEndTime, int& rStartFrame, int& rEndFrame) const;
    //void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
    const std::string& getFileName() const { return _fileName; }
    const OrientationSensorFrame& getFrame(int aIndex) const;
    int getOrientationSensorIndex(const std::string& aName) const;
    const Units& getUnits() const { return _units; }
    const Array<std::string>& getOrientationSensorNames() const { return _orientationSensorNames; }
    int getNumOrientationSensors() const { return _numOrientationSensors; }
    int getNumFrames() const { return _numFrames; }
    double getStartFrameTime() const;
    double getLastFrameTime() const;
    double getDataRate() const { return _dataRate; }
    double getCameraRate() const { return _cameraRate; } // TODO check if these info are useless and can be removed

private:
//    void readMOTFile(const std::string& aFileName, OrientationSensorData& aSMD);
//    void readMOTFileHeader(std::ifstream &in, const std::string& aFileName, OrientationSensorData& aSMD);
    void readStoFile(const std::string& aFileName);
    void buildOSensorMap(const Storage& storageToReadFrom, std::map<int, std::string>& orientationSensorNames);

//=============================================================================
};  // END of class OrientationSensorData
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __OrientationSensorData_h__


