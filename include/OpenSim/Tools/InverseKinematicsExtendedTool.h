/* -------------------------------------------------------------------------- *
 *   Orientation Based Inverse Kinematics : InverseKinematicsExtendedTool.h   *
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

#ifndef IKVERSE_KINEMATICS_EXTENDED_TOOL_H_
#define IKVERSE_KINEMATICS_EXTENDED_TOOL_H_

#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include "OpenSim/Tools/osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include "OpenSim/Tools/Tool.h"

#ifdef SWIG
    #ifdef OSIMEXTENDEDIK_API
        #undef OSIMEXTENDEDIK_API
        #define OSIMEXTENDEDIK_API
    #endif
#endif

namespace OpenSim {

class Model;
class IKExtendedTaskSet;
class Storage;

class OSIMEXTENDEDIK_API InverseKinematicsExtendedTool : public Tool{
OpenSim_DECLARE_CONCRETE_OBJECT(InverseKinematicsExtendedTool, Tool);

private:

    /** Pointer to the model being investigated. */
    Model *_model;

    /** Name of the xml file used to deserialize or construct a model. */
    PropertyStr _modelFileNameProp;
    std::string &_modelFileName;

    /** The relative weight of the constraints. Infinity is a strictly enforced constraint.
        Any other non-zero positive scalar is the penalty factor for constraint violations. */
    PropertyDbl _constraintWeightProp;
    double &_constraintWeight;

    /** The accuracy of the solution in absolute terms, i.e. the number of significant digits
        to which the solution can be trusted. */
    PropertyDbl _accuracyProp;
    double &_accuracy;

    // Markers and coordinates to be matched and their respective weightings
    PropertyObj _ikExtendedTaskSetProp;
    IKExtendedTaskSet &_ikExtendedTaskSet;

    // name of marker file that contains marker locations for IK solving
    PropertyStr _markerFileNameProp;
    std::string &_markerFileName;

    // name of marker file that contains marker locations for IK solving
    PropertyStr _oSensorFileNameProp;
    std::string &_oSensorFileName;

    // name of storage file that contains coordinate values for IK solving
    PropertyStr _coordinateFileNameProp;
    std::string &_coordinateFileName;

    // range of frames to solve in marker file, specified by time
    PropertyDblArray _timeRangeProp;
    Array<double> &_timeRange;

    // flag if inverse kinematics should report marker errors
    PropertyBool _reportErrorsProp;
    bool &_reportErrors;

    // name of motion file with inverse kinematics solution
    PropertyStr _outputMotionFileNameProp;
    std::string &_outputMotionFileName;

    // flag indicating whether or not to resulting marker locations
    PropertyBool _reportMarkerLocationsProp;
    bool &_reportMarkerLocations;

public:
    virtual ~InverseKinematicsExtendedTool();
    InverseKinematicsExtendedTool();
    InverseKinematicsExtendedTool(const std::string &aFileName, bool aLoadModel = true) SWIG_DECLARE_EXCEPTION;
    InverseKinematicsExtendedTool(const InverseKinematicsExtendedTool &aObject);

    /* Register types to be used when reading an InverseKinematicsExtendedTool object from xml file. */
    static void registerTypes();
    /* Handle reading older formats/Versioning */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

    //---- Setters and getters for various attributes
    void setModel(Model& aModel) { _model = &aModel; };
    void setStartTime(double d) { _timeRange[0] = d; };
    double getStartTime() const {return  _timeRange[0]; };

    void setEndTime(double d) { _timeRange[1] = d; };
    double getEndTime() const {return  _timeRange[1]; };

    void setMarkerDataFileName(const std::string& markerDataFileName) { _markerFileName=markerDataFileName;};
    const std::string& getMarkerDataFileName() const { return  _markerFileName;};

    void setOSensorDataFileName(const std::string& markerDataFileName) { _oSensorFileName = markerDataFileName; };
    const std::string& getOSensorDataFileName() const { return  _oSensorFileName; };

    void setCoordinateFileName(const std::string& coordDataFileName) { _coordinateFileName=coordDataFileName;};
    const std::string& getCoordinateFileName() const { return  _coordinateFileName;};

    bool getUseVisualizer() {return useVisualizer_;};
    void setUseVisualizer(bool useVisualizer) {useVisualizer_ = useVisualizer;};

    //const OpenSim::Storage& getOutputStorage() const;
private:
    void setNull();
    void setupProperties();

    bool hasMarkerFile_ = false;
    bool hasOSensorFile_ = false;
    bool useVisualizer_ = false;

public:
#ifndef SWIG
    InverseKinematicsExtendedTool&
        operator=(const InverseKinematicsExtendedTool &aInverseKinematicsTool);
#endif

    void setOutputMotionFileName(const std::string aOutputMotionFileName) {
        _outputMotionFileName = aOutputMotionFileName;
    }
    std::string getOutputMotionFileName() { return _outputMotionFileName;}
    IKExtendedTaskSet& getIKTaskSet() { return _ikExtendedTaskSet; }

    bool run() override SWIG_DECLARE_EXCEPTION;

};  // END of class InverseKinematicsExtendedTool

} // namespace

#endif // IKVERSE_KINEMATICS_EXTENDED_TOOL_H_
