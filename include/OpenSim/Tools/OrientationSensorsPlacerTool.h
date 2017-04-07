/* -------------------------------------------------------------------------- *
 *    Orientation Based Inverse Kinematics: OrientationSensorsPlacerTool.h    *
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

#ifndef __OrientationSensorsPlacerTool_h__
#define __OrientationSensorsPlacerTooll_h__

#include <iostream>
#include <math.h>

#include <OpenSim/Simulation/osimExtendedIKDLL.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "OpenSim/Tools/OrientationSensorPlacer.h"
namespace OpenSim {

class OSIMEXTENDEDIK_API OrientationSensorsPlacerTool : public Object {
  OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorsPlacerTool, Object);

  protected:
    PropertyDbl _massProp;
    double &_mass;

    PropertyDbl _heightProp;
    double &_height;

    PropertyDbl _ageProp;
    double &_age;

    PropertyStr _notesProp;
    std::string &_notes;

    PropertyStr _modelFileNameProp;
    std::string &_modelFileName;

    // PropertyStr _oSensorSetFileNameProp;
    // std::string &_oSensorSetFileName;

    PropertyObj _oSensorPlacerProp;
    OrientationSensorPlacer &_oSensorPlacer;

    /** All files in workflow are specified relative to
     * where the subject file is. Need to keep track of that in case absolute
     * path is needed later
     */
    std::string	 _pathToSubject;

  public:
    OrientationSensorsPlacerTool();
    OrientationSensorsPlacerTool(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;
    OrientationSensorsPlacerTool(const OrientationSensorsPlacerTool &aSubject);
    virtual ~OrientationSensorsPlacerTool();

#ifndef SWIG
    OrientationSensorsPlacerTool& operator=(const OrientationSensorsPlacerTool &aSubject);
#endif
    void copyData(const OrientationSensorsPlacerTool &aSubject);

    Model* createModel();

    OrientationSensorPlacer& getOrientationSensorPlacer() { return _oSensorPlacer; }

    bool isDefaultOrientationSensorPlacer() { return _oSensorPlacerProp.getValueIsDefault(); }

    static void registerTypes();

    /** Accessor methods to obtain model attributes */
    double getSubjectMass() { return _mass; }
    double getSubjectAge() { return _age; }
    double getSubjectHeight() { return _height; }
    void setSubjectMass(double mass) { _mass = mass; }
    void setSubjectAge(double age) { _age = age; }
    void setSubjectHeight(double height) { _height = height; }

    /**
     * Accessor methods to set and get path to Subject. This is needed
     * since all file names referred to in the subject file are relative
     * to subject file
     */
    const std::string& getPathToSubject() { return _pathToSubject; }
    void setPathToSubject(const std::string& aPath) { _pathToSubject = aPath; }

    void setPrintResultFiles(bool aToWrite) { _oSensorPlacer.setPrintResultFiles(aToWrite); }

  private:
    void setNull();
    void setupProperties();

  };	// end of class OrientationSensorsPlacerTool

} // end of namespace OpenSim

#endif // __OrientationSensorsPlacerTool_h__
