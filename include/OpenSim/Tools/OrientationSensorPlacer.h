/* -------------------------------------------------------------------------- *
 *     Orientation Based Inverse Kinematics: OrientationSensorsPlacer.cpp     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2016-2017 L. Tagliapietra, E. Ceseracciu, M. Reggiani        *
 *                                                                            *
 * Author(s): L. Tagliapietra, E. Ceseracciu (Dec 2016)                       *
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

#ifndef __OrientationSensorPlacer_h__
#define __OrientationSensorPlacer_h__

#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/osimExtendedIKDLL.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>

#include "IKExtendedTaskSet.h"
#include <SimTKsimbody.h>

namespace OpenSim {

  class Model;
  class OrientationSensorData;
  class IKTrial;
  class Storage;

  //=============================================================================
  //=============================================================================
  /**
   * A class implementing a set of parameters describing how to place markers
   * on a model (presumably after it has been scaled to fit a subject).
   *
   * @author Peter Loan
   * @version 1.0
   */

   //=============================================================================
   //=============================================================================
   /**
   * OrientationSensorPlacer.h
   * The extended version of the original OpenSim MarkerPlacer class to allow orientation
   * sensor orientations adjustments on body segments during a static trial
   *
   * @author: Luca Tagliapietra <tagliapietra@gest.unipd.it>
   * @notes: 2016, Dec
   */

  class OSIMEXTENDEDIK_API OrientationSensorPlacer : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorPlacer, Object);

    //=============================================================================
    // DATA
    //=============================================================================
  private:

  protected:
    // whether or not to apply orientation sensor placer
    PropertyBool _applyProp;
    bool &_apply;

    // name of orientation sensor file that contains oSensor orientations in the static pose
    PropertyStr _oSensorFileNameProp;
    std::string &_oSensorFileName;

    // range of frames to average in static pose oSensor file, specified by time
    PropertyDblArray _timeRangeProp;
    Array<double> &_timeRange;

    // tasks used to specify IK weights
    PropertyObj _ikExtendedTaskSetProp;
    IKExtendedTaskSet &_ikExtendedTaskSet;

    // name of SIMM motion file that contains [optional] coordinates for the static pose
    PropertyStr _coordinateFileNameProp;
    std::string &_coordinateFileName;

    // name of XML model file to write when done placing oSensors
    PropertyStr _outputModelFileNameProp;
    std::string &_outputModelFileName;

    // name of oSensor file to write when done placing oSensors
    PropertyStr _outputOSensorFileNameProp;
    std::string &_outputOSensorFileName;

    // name of motion file (containing solved static pose) when done placing oSensors
    PropertyStr _outputMotionFileNameProp;
    std::string &_outputMotionFileName;

    // amount of allowable motion for each oSensor when averaging frames of the static trial
    PropertyDbl _maxOSensorMovementProp;
    double &_maxOSensorMovement;

    // Whether or not to write write to the designated output files (GUI will set this to false)
    bool _printResultFiles;
    // Whether to move the model oSensors (set to false if you just want to preview the static pose)
    bool _moveModelOSensors;

    Storage* _outputStorage;
    //=============================================================================
    // METHODS
    //=============================================================================
      //--------------------------------------------------------------------------
      // CONSTRUCTION
      //--------------------------------------------------------------------------
  public:
    OrientationSensorPlacer();
    OrientationSensorPlacer(const OrientationSensorPlacer &anOrientationSensorPlacementParams);
    virtual ~OrientationSensorPlacer();

    void copyData(const OrientationSensorPlacer &anOrientationSensorPlacementParams);

#ifndef SWIG
    OrientationSensorPlacer& operator=(const OrientationSensorPlacer &anOrientationSensorPlacementParams);
#endif
    bool processModel(SimTK::State& s, Model* aModel, const std::string& aPathToSubject = "");

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool getApply() const { return _apply; }
    void setApply(bool aApply)
    {
      _apply = aApply;
      _applyProp.setValueIsDefault(false);
    }

    const std::string &getStaticPoseFileName() const { return _oSensorFileName; }
    void setStaticPoseFileName(const std::string &aFileName)
    {
      _oSensorFileName = aFileName;
      _oSensorFileNameProp.setValueIsDefault(false);
    }

    const Array<double> &getTimeRange() const { return _timeRange; }
    void setTimeRange(const Array<double> &timeRange)
    {
      _timeRange = timeRange;
      _timeRangeProp.setValueIsDefault(false);
    }

    IKExtendedTaskSet &getIKExtendedTaskSet() { return _ikExtendedTaskSet; }

    const std::string &getCoordinateFileName() const { return _coordinateFileName; }
    void setCoordinateFileName(const std::string& aCoordinateFileName)
    {
      _coordinateFileName = aCoordinateFileName;
      _coordinateFileNameProp.setValueIsDefault(false);
    }

    double getMaxOSensorMovement() const { return _maxOSensorMovement; }
    void setMaxOSensorMovement(double aMaxOSensorMovement)
    {
      _maxOSensorMovement = aMaxOSensorMovement;
      _maxOSensorMovementProp.setValueIsDefault(false);
    }

    const std::string& getOutputModelFileName() const { return _outputModelFileName; }
    void setOutputModelFileName(const std::string& aOutputModelFileName)
    {
      _outputModelFileName = aOutputModelFileName;
      _outputModelFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputOSensorFileName() const { return _outputOSensorFileName; }
    void setOutputOSensorFileName(const std::string& outputOSensorFileName)
    {
      _outputOSensorFileName = outputOSensorFileName;
      _outputOSensorFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputMotionFileName() const { return _outputMotionFileName; }
    void setOutputMotionFileName(const std::string& outputMotionFileName)
    {
      _outputMotionFileName = outputMotionFileName;
      _outputMotionFileNameProp.setValueIsDefault(false);
    }

    void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

    bool getMoveModelOSensors() { return _moveModelOSensors; }
    void setMoveModelOSensors(bool aMove) { _moveModelOSensors = aMove; }

    Storage *getOutputStorage();


  private:
    void setNull();
    void setupProperties();
    void moveModelOSensorsToPose(SimTK::State& s, Model& aModel, OrientationSensorData& aPose);
    //=============================================================================
  };	// END of class OrientationSensorPlacer
  //=============================================================================
  //=============================================================================

} // end of namespace OpenSim

#endif // __OrientationSensorPlacer_h__
