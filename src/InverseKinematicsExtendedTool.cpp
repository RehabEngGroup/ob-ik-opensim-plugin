/* -------------------------------------------------------------------------- *
 *                    OpenSim:  InverseKinematicsTool.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsExtendedSolver.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/XMLDocument.h>

#include <OpenSim/Analyses/Kinematics.h>

#include "OpenSim/Tools/IKExtendedTaskSet.h"
#include "OpenSim/Tools/IKCoordinateTask.h"
#include "OpenSim/Tools/IKMarkerTask.h"
#include "OpenSim/Tools/IKOrientationSensorTask.h"

#include "SimTKsimbody.h"


using namespace OpenSim;
using namespace std;
using namespace SimTK;

inline bool isAbsolute(const char *path) {
  if (path[0] == '/' || path[0] == '\\') {
    return true;
  }
  std::string str(path);
  if (str.length()>1) {
    if (str[1] == ':') {
      return true;
    }
  }
  return false;
};


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InverseKinematicsExtendedTool::~InverseKinematicsExtendedTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseKinematicsExtendedTool::InverseKinematicsExtendedTool() : Tool(),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _constraintWeight(_constraintWeightProp.getValueDbl()),
    _accuracy(_accuracyProp.getValueDbl()),
    _ikExtendedTaskSetProp(PropertyObj("", IKExtendedTaskSet())),
    _ikExtendedTaskSet((IKExtendedTaskSet&)_ikExtendedTaskSetProp.getValueObj()),
    _markerFileName(_markerFileNameProp.getValueStr()),
    _oSensorFileName(_oSensorFileNameProp.getValueStr()),
    _coordinateFileName(_coordinateFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _reportErrors(_reportErrorsProp.getValueBool()),
    _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
    _reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
InverseKinematicsExtendedTool::InverseKinematicsExtendedTool(const string &aFileName, bool aLoadModel) :
    Tool(aFileName, true),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _constraintWeight(_constraintWeightProp.getValueDbl()),
    _accuracy(_accuracyProp.getValueDbl()),
    _ikExtendedTaskSetProp(PropertyObj("", IKExtendedTaskSet())),
    _ikExtendedTaskSet((IKExtendedTaskSet&)_ikExtendedTaskSetProp.getValueObj()),
    _markerFileName(_markerFileNameProp.getValueStr()),
    _oSensorFileName(_oSensorFileNameProp.getValueStr()),
    _coordinateFileName(_coordinateFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _reportErrors(_reportErrorsProp.getValueBool()),
    _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
    _reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
    setNull();
    updateFromXMLDocument();
    std::string pathToSetupFile = IO::getParentDirectory(aFileName);

    if (!isAbsolute(_modelFileName.c_str()))
      _modelFileName = pathToSetupFile + _modelFileName;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTool Object to be copied.

 */
InverseKinematicsExtendedTool::InverseKinematicsExtendedTool(const InverseKinematicsExtendedTool &aTool) :
    Tool(aTool),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _constraintWeight(_constraintWeightProp.getValueDbl()),
    _accuracy(_accuracyProp.getValueDbl()),
    _ikExtendedTaskSetProp(PropertyObj("", IKExtendedTaskSet())),
    _ikExtendedTaskSet((IKExtendedTaskSet&)_ikExtendedTaskSetProp.getValueObj()),
    _markerFileName(_markerFileNameProp.getValueStr()),
    _oSensorFileName(_oSensorFileNameProp.getValueStr()),
    _coordinateFileName(_coordinateFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblArray()),
    _reportErrors(_reportErrorsProp.getValueBool()),
    _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
    _reportMarkerLocations(_reportMarkerLocationsProp.getValueBool())
{
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InverseKinematicsExtendedTool::setNull()
{
    setupProperties();
    _model = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseKinematicsExtendedTool::setupProperties()
{
    _modelFileNameProp.setComment("Name of the .osim file used to construct a model.");
    _modelFileNameProp.setName("model_file");
    _propertySet.append( &_modelFileNameProp );

    _constraintWeightProp.setComment("A positive scalar that is used to weight the importance of satisfying constraints."
        "A weighting of 'Infinity' or if it is unassigned results in the constraints being strictly enforced.");
    _constraintWeightProp.setName("constraint_weight");
    _constraintWeightProp.setValue(std::numeric_limits<SimTK::Real>::infinity());
    _propertySet.append( &_constraintWeightProp );

    _accuracyProp.setComment("The accuracy of the solution in absolute terms. I.e. the number of significant"
        "digits to which the solution can be trusted.");
    _accuracyProp.setName("accuracy");
    _accuracyProp.setValue(1e-5);
    _propertySet.append( &_accuracyProp );

    _ikExtendedTaskSetProp.setComment("Markers, Orientation Sensors, and coordinates to be considered (tasks) and their weightings.");
    _ikExtendedTaskSetProp.setName("IKExtendedTaskSet");
    _propertySet.append(&_ikExtendedTaskSetProp);

    _markerFileNameProp.setComment("TRC file (.trc) containing the time history of observations of marker positions.");
    _markerFileNameProp.setName("marker_file");
    _propertySet.append(&_markerFileNameProp);

    _oSensorFileNameProp.setComment("MOT/STO file (.mot/.sto) containing the time history of observations of orientation sensor pose.");
    _oSensorFileNameProp.setName("orientation_sensor_file");
    _propertySet.append(&_oSensorFileNameProp);

    _coordinateFileNameProp.setComment("The name of the storage (.sto or .mot) file containing coordinate observations."
        "Coordinate values from this file are included if there is a corresponding coordinate task. ");
    _coordinateFileNameProp.setName("coordinate_file");
    _propertySet.append(&_coordinateFileNameProp);

    const double defaultTimeRange[] = {-std::numeric_limits<SimTK::Real>::infinity(), std::numeric_limits<SimTK::Real>::infinity()};
    _timeRangeProp.setComment("Time range over which the inverse kinematics problem is solved.");
    _timeRangeProp.setName("time_range");
    _timeRangeProp.setValue(2, defaultTimeRange);
    _timeRangeProp.setAllowableListSize(2);
    _propertySet.append(&_timeRangeProp);

    _reportErrorsProp.setComment("Flag (true or false) indicating whether or not to report marker "
        "errors from the inverse kinematics solution.");
    _reportErrorsProp.setName("report_errors");
    _reportErrorsProp.setValue(true);
    _propertySet.append(&_reportErrorsProp);

    _outputMotionFileNameProp.setComment("Name of the motion file (.mot) to which the results should be written.");
    _outputMotionFileNameProp.setName("output_motion_file");
    _propertySet.append(&_outputMotionFileNameProp);

    _reportMarkerLocationsProp.setComment("Flag indicating whether or not to report model marker locations in ground.");
    _reportMarkerLocationsProp.setName("report_marker_locations");
    _reportMarkerLocationsProp.setValue(false);
    _propertySet.append(&_reportMarkerLocationsProp);

}

//_____________________________________________________________________________
/**
 * Register InverseKinematicsTool and any Object types it may employ internally.
 */
void InverseKinematicsExtendedTool::registerTypes()
{
    Object::registerType(InverseKinematicsExtendedTool());
}
//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
InverseKinematicsExtendedTool& InverseKinematicsExtendedTool::
operator=(const InverseKinematicsExtendedTool &aTool)
{
    // BASE CLASS
    Tool::operator=(aTool);

    // MEMBER VARIABLES
    _modelFileName = aTool._modelFileName;
    _constraintWeight = aTool._constraintWeight;
    _accuracy = aTool._accuracy;
    _ikExtendedTaskSet = aTool._ikExtendedTaskSet;
    _markerFileName = aTool._markerFileName;
    _oSensorFileName = aTool._oSensorFileName;
    _timeRange = aTool._timeRange;
    _reportErrors = aTool._reportErrors;
    _coordinateFileName = aTool._coordinateFileName;
    _reportErrors = aTool._reportErrors;
    _outputMotionFileName = aTool._outputMotionFileName;
    _reportMarkerLocations = aTool._reportMarkerLocations;

    return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the inverse kinematics tool.
 */
bool InverseKinematicsExtendedTool::run()
{
    bool success = false;
    bool modelFromFile=true;
    try{
        //Load and create the indicated model
        if (!_model)
            _model = new Model(_modelFileName);
        else
            modelFromFile = false;

        _model->printBasicInfo(std::cout);
        //TODO: Add this as parameter from xml
        _model->setUseVisualizer(true);
        // Do the maneuver to change then restore working directory
        // so that the parsing code behaves properly if called from a different directory.
        string saveWorkingDirectory = IO::getCwd();
        string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
        IO::chDir(directoryOfSetupFile);

        // Define reporter for output
        Kinematics kinematicsReporter;
        kinematicsReporter.setRecordAccelerations(true);
        kinematicsReporter.setInDegrees(true);
        _model->addAnalysis(&kinematicsReporter);

        std::cout<<"Running tool "<<getName()<<".\n";

        // Get the trial name to label data written to files
        string trialName = getName();

        // Initialize the model's underlying computational system and get its default state.
        SimTK::State& s = _model->initSystem();

        //Convert old Tasks to references for assembly and tracking
        MarkersReference markersReference;
        Set<MarkerWeight> markerWeights;
        OrientationSensorsReference oSensorsReference;
        Set<OrientationSensorWeight> oSensorWeights;
        SimTK::Array_<CoordinateReference> coordinateReferences;

        FunctionSet *coordFunctions = NULL;
        // Load the coordinate data
        // bool haveCoordinateFile = false;
        if(_coordinateFileName != "" && _coordinateFileName != "Unassigned"){
            Storage coordinateValues(_coordinateFileName);
            // Convert degrees to radian (TODO: this needs to have a check that the storage is, in fact, in degrees!)
            _model->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
            // haveCoordinateFile = true;
            coordFunctions = new GCVSplineSet(5,&coordinateValues);
        }

        if (_markerFileName != "" && _markerFileName != "Unassigned")
          hasMarkerFile_ = true;
        if (_oSensorFileName != "" && _oSensorFileName != "Unassigned")
          hasOSensorFile_ = true;

        // Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
        // For coordinates, create the functions for coordinate reference values
        int index = 0;
        for(int i=0; i < _ikExtendedTaskSet.getSize(); i++){
            if (!_ikExtendedTaskSet[i].getApply()) continue;
            if(IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&_ikExtendedTaskSet[i])){
                CoordinateReference *coordRef = NULL;
                if(coordTask->getValueType() == IKCoordinateTask::FromFile){
                     if (!coordFunctions)
                        throw Exception("InverseKinematicsExtendedTool: value for coordinate "+coordTask->getName()+" not found.");

                     index = coordFunctions->getIndex(coordTask->getName(), index);
                     if(index >= 0){
                         coordRef = new CoordinateReference(coordTask->getName(),coordFunctions->get(index));
                     }
                }
                else if((coordTask->getValueType() == IKCoordinateTask::ManualValue)){
                        Constant reference(Constant(coordTask->getValue()));
                        coordRef = new CoordinateReference(coordTask->getName(), reference);
                }
                else{ // assume it should be held at its default value
                    double value = _model->getCoordinateSet().get(coordTask->getName()).getDefaultValue();
                    Constant reference = Constant(value);
                    coordRef = new CoordinateReference(coordTask->getName(), reference);
                }

                if(coordRef == NULL)
                    throw Exception("InverseKinematicsExtendedTool: value for coordinate "+coordTask->getName()+" not found.");
                else
                    coordRef->setWeight(coordTask->getWeight());

                coordinateReferences.push_back(*coordRef);
            }
            else if(IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&_ikExtendedTaskSet[i])){
                if(markerTask->getApply()){
                    markerWeights.adoptAndAppend(
                        new MarkerWeight(markerTask->getName(), markerTask->getWeight()));
                }
            }
            else if (IKOrientationSensorTask *oSensorTask = dynamic_cast<IKOrientationSensorTask *>(&_ikExtendedTaskSet[i])){
                if (oSensorTask->getApply()){
                    oSensorWeights.adoptAndAppend(
                        new OrientationSensorWeight(oSensorTask->getName(), oSensorTask->getWeight()));
                }
            }
        }

        double start_time = _timeRange[0];
        double final_time = _timeRange[1];
        double dt = SimTK::Infinity;

        if (hasMarkerFile_) {
          markersReference.setMarkerWeightSet(markerWeights);
          markersReference.loadMarkersFile(_markerFileName);
          SimTK::Vec2 markersValidTimRange = markersReference.getValidTimeRange();
          start_time = (markersValidTimRange[0] > start_time) ? markersValidTimRange[0] : start_time;
          final_time = (markersValidTimRange[1] < final_time) ? markersValidTimRange[1] : final_time;
          dt = (dt > 1.0 / markersReference.getSamplingFrequency()) ? 1.0 / markersReference.getSamplingFrequency() : dt;
        }
        if (hasOSensorFile_) {
          oSensorsReference.setOrientationSensorWeightSet(oSensorWeights);
          oSensorsReference.loadOrientationSensorsFile(_oSensorFileName);
          SimTK::Vec2 oSensorsValidTimRange = oSensorsReference.getValidTimeRange();
          start_time = (oSensorsValidTimRange[0] > start_time) ? oSensorsValidTimRange[0] : start_time;
          final_time = (oSensorsValidTimRange[1] < final_time) ? oSensorsValidTimRange[1] : final_time;
          dt = (dt > 1.0 / oSensorsReference.getSamplingFrequency()) ? 1.0 / oSensorsReference.getSamplingFrequency() : dt;
        }
        InverseKinematicsExtendedSolver * ikSolver;
        if (hasMarkerFile_ && hasOSensorFile_)
          ikSolver = new InverseKinematicsExtendedSolver(*_model, markersReference, oSensorsReference, coordinateReferences, _constraintWeight);
        else if (hasMarkerFile_ && !hasOSensorFile_)
          ikSolver = new InverseKinematicsExtendedSolver(*_model, markersReference, coordinateReferences, _constraintWeight);
        else if (!hasMarkerFile_ && hasOSensorFile_)
          ikSolver = new InverseKinematicsExtendedSolver(*_model, oSensorsReference, coordinateReferences, _constraintWeight);
        else
          return 0;

        ikSolver->setAccuracy(_accuracy);
        ikSolver->setSerializeAllDefaults(true);
        std::cout << ikSolver->dump(true) << std::endl;

        s.updTime() = start_time;
        int Nframes = int((final_time - start_time) / dt) + 1;

        ikSolver->assemble(s);
        kinematicsReporter.begin(s);

        const clock_t start = clock();
        AnalysisSet& analysisSet = _model->updAnalysisSet();
        analysisSet.begin(s);

        // number of markers
        int nm = markerWeights.getSize();
        int nos = oSensorWeights.getSize();
        SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
        SimTK::Array_<double> oSensorErrors(nos, 0.0);

        // TODO: check if useful also for orientation sensors
        SimTK::Array_<Vec3> markerLocations(nm, Vec3(0));

        Storage *modelMarkerLocations = _reportMarkerLocations ? new Storage(Nframes, "ModelMarkerLocations") : NULL;
        Storage *modelMarkerErrors = _reportErrors ? new Storage(Nframes, "ModelMarkerErrors") : NULL;
      //  Storage *modelOSensorErrors = _reportErrors ? new Storage(Nframes, "ModelOsensorErrors") : NULL; //TODO print file if requested

        for (int i = 0; i < Nframes; i++) {
            s.updTime() = start_time + i*dt;
            ikSolver->track(s);
            if (_model->hasVisualizer()) {
              auto& viz = _model->updVisualizer().updSimbodyVisualizer();
              viz.setBackgroundType(viz.GroundAndSky);
              viz.setShowSimTime(true);
              viz.report(s);
            }
            std::cout << "Tracking at " << i << std::endl;
            if (_reportErrors) {
              std::cout << "Frame " << i << " (t=" << s.getTime() << "):\t";

              if (nm > 0) {
                Array<double> markerErrors(0.0, 3);
                double totalSquaredMarkerError = 0.0;
                double maxSquaredMarkerError = 0.0;
                int worstMarker = -1;
                ikSolver->computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
                for (int j = 0; j < nm; ++j) {
                  totalSquaredMarkerError += squaredMarkerErrors[j];
                  if (squaredMarkerErrors[j] > maxSquaredMarkerError) {
                    maxSquaredMarkerError = squaredMarkerErrors[j];
                    worstMarker = j;
                  }
                }
                double rms = sqrt(totalSquaredMarkerError / nm);
                markerErrors.set(0, totalSquaredMarkerError);
                markerErrors.set(1, rms);
                markerErrors.set(2, sqrt(maxSquaredMarkerError));
                modelMarkerErrors->append(s.getTime(), 3, &markerErrors[0]);
                std::cout << "total squared marker error = " << totalSquaredMarkerError;
                std::cout << ", marker error: RMS=" << rms;
                std::cout << ", max=" << sqrt(maxSquaredMarkerError) << " (" << ikSolver->getMarkerNameForIndex(worstMarker) << ")" << std::endl;

                if (_reportMarkerLocations) {
                  ikSolver->computeCurrentMarkerLocations(markerLocations);
                  Array<double> locations(0.0, 3 * nm);
                  for (int j = 0; j < nm; ++j) {
                    for (int k = 0; k < 3; ++k)
                      locations.set(3 * j + k, markerLocations[j][k]);
                  }
                  modelMarkerLocations->append(s.getTime(), 3 * nm, &locations[0]);
                }
              }

              // Report oSensor errors to assess the quality
              if(nos>0) {

                SimTK::Array_<double> oSensorErrors(nos, 0.0);
                double totalError = 0.0;
                double maxOSensorError = 0.0;
                double squaredSum = 0.0;
                int worstOSensor = -1;

                ikSolver->computeCurrentOSensorErrors(oSensorErrors);

                for (int j = 0; j < nos; ++j) {
                  totalError +=oSensorErrors[j];
                  squaredSum += SimTK::square(oSensorErrors[j]);
                  if (oSensorErrors[j] > maxOSensorError) {
                    maxOSensorError = oSensorErrors[j];
                    worstOSensor = j;
                  }
                }
                double rmsError = sqrt(squaredSum / nos);

                cout << "OSensors tracking:\t";
                cout << "total absolute angular error = " << SimTK::convertRadiansToDegrees(totalError) << " degs";
                cout << ", RMS angular error = " << SimTK::convertRadiansToDegrees(rmsError) << " degs";
                cout << ", max absolute angular error = " << SimTK::convertRadiansToDegrees(maxOSensorError) << " degs ( " << ikSolver->getOSensorNameForIndex(worstOSensor) << " )" << endl;

              }
            }
            kinematicsReporter.step(s, i);
            analysisSet.step(s, i);
        }

        // Do the maneuver to change then restore working directory
        // so that output files are saved to same folder as setup file.
        if (_outputMotionFileName!= "" && _outputMotionFileName!="Unassigned"){
            kinematicsReporter.getPositionStorage()->print(_outputMotionFileName);
        }

        if (nm>0 && modelMarkerErrors) {
            Array<string> labels("", 4);
            labels[0] = "time";
            labels[1] = "total_squared_error";
            labels[2] = "marker_error_RMS";
            labels[3] = "marker_error_max";

            modelMarkerErrors->setColumnLabels(labels);
            modelMarkerErrors->setName("Model Marker Errors from IK");

            IO::makeDir(getResultsDir());
            string errorFileName = trialName + "_ik_marker_errors";
            Storage::printResult(modelMarkerErrors, errorFileName, getResultsDir(), -1, ".sto");

            delete modelMarkerErrors;
        }

        if(nm >0 && modelMarkerLocations){
            Array<string> labels("", 3*nm+1);
            labels[0] = "time";
            Array<string> XYZ("", 3*nm);
            XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

            for(int j=0; j<nm; ++j){
                for(int k=0; k<3; ++k)
                    labels.set(3*j+k+1, ikSolver->getMarkerNameForIndex(j)+XYZ[k]);
            }
            modelMarkerLocations->setColumnLabels(labels);
            modelMarkerLocations->setName("Model Marker Locations from IK");

            IO::makeDir(getResultsDir());
            string markerFileName = trialName + "_ik_model_marker_locations";
            Storage::printResult(modelMarkerLocations, markerFileName, getResultsDir(), -1, ".sto");

            delete modelMarkerLocations;
        }

        IO::chDir(saveWorkingDirectory);
        success = true;
        std::cout << "InverseKinematicsTool completed " << Nframes-1 << " frames in " <<(double)(clock()-start)/CLOCKS_PER_SEC << "s\n" << std::endl;
    }
    catch (const std::exception& ex) {
        std::cout << "InverseKinematicsTool Failed: " << ex.what() << std::endl;
        throw (Exception("InverseKinematicsTool Failed, please see messages window for details..."));
    }

    if (modelFromFile) delete _model;

    return success;
}

// Handle conversion from older format
void InverseKinematicsExtendedTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if ( versionNumber < XMLDocument::getLatestVersion()){
        std::string newFileName = getDocumentFileName();
        if (versionNumber < 20300){
            std::string origFilename = getDocumentFileName();
            newFileName=IO::replaceSubstring(newFileName, ".xml", "_v23.xml");
            std::cout << "Old version setup file encountered. Converting to new file "<< newFileName << std::endl;
            SimTK::Xml::Document doc = SimTK::Xml::Document(origFilename);
            doc.writeToFile(newFileName);
        }
        if (versionNumber <= 20201){
            // get filename and use SimTK::Xml to parse it
            SimTK::Xml::Document doc = SimTK::Xml::Document(newFileName);
            Xml::Element root = doc.getRootElement();
            if (root.getElementTag()=="OpenSimDocument"){
                int curVersion = root.getRequiredAttributeValueAs<int>("Version");
                if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
                Xml::element_iterator iter(root.element_begin("IKTool"));
                iter->setElementTag("InverseKinematicsTool");
                Xml::element_iterator toolIter(iter->element_begin("IKTrialSet"));
                // No optimizer_algorithm specification anymore
                Xml::element_iterator optIter(iter->element_begin("optimizer_algorithm"));
                if (optIter!= iter->element_end())
                    iter->eraseNode(optIter);

                Xml::element_iterator objIter(toolIter->element_begin("objects"));
                Xml::element_iterator trialIter(objIter->element_begin("IKTrial"));
                // Move children of (*trialIter) to root
                Xml::node_iterator p = trialIter->node_begin();
                for (; p!= trialIter->node_end(); ++p) {
                    iter->insertNodeAfter( iter->node_end(), p->clone());
            }
                // Append constraint_weight of 100 and accuracy of 1e-5
                iter->insertNodeAfter( iter->node_end(), Xml::Comment(_constraintWeightProp.getComment()));
                iter->insertNodeAfter( iter->node_end(), Xml::Element("constraint_weight", "20.0"));
                iter->insertNodeAfter( iter->node_end(), Xml::Comment(_accuracyProp.getComment()));
                iter->insertNodeAfter( iter->node_end(), Xml::Element("accuracy", "1e-4"));
                // erase node for IKTrialSet
                iter->eraseNode(toolIter);
                Xml::Document newDocument;
                Xml::Element docElement= newDocument.getRootElement();
                docElement.setAttributeValue("Version", "20300");
                docElement.setElementTag("OpenSimDocument");
                // Copy all children of root to newRoot
                docElement.insertNodeAfter(docElement.node_end(), iter->clone());
                newDocument.writeToFile(newFileName);
                setDocument(new XMLDocument(newFileName));
                aNode = updDocument()->getRootDataElement();
            }
            else {
                if (root.getElementTag()=="IKTool"){
                    root.setElementTag("InverseKinematicsTool");
                    Xml::element_iterator toolIter(root.element_begin("IKTrialSet"));
                    if (toolIter== root.element_end())
                        throw (Exception("Old IKTool setup file doesn't have required IKTrialSet element.. Aborting"));
                    // No optimizer_algorithm specification anymore
                    Xml::element_iterator optIter(root.element_begin("optimizer_algorithm"));
                    if (optIter!= root.element_end())
                    root.eraseNode(optIter);

                    Xml::element_iterator objIter(toolIter->element_begin("objects"));
                    Xml::element_iterator trialIter(objIter->element_begin("IKTrial"));
                    // Move children of (*trialIter) to root
                    Xml::node_iterator p = trialIter->node_begin();
                    for (; p!= trialIter->node_end(); ++p) {
                        root.insertNodeAfter( root.node_end(), p->clone());
                    }
                    // Append constraint_weight of 100 and accuracy of 1e-5
                    root.insertNodeAfter( root.node_end(), Xml::Comment(_constraintWeightProp.getComment()));
                    root.insertNodeAfter( root.node_end(), Xml::Element("constraint_weight", "20.0"));
                    root.insertNodeAfter( root.node_end(), Xml::Comment(_accuracyProp.getComment()));
                    root.insertNodeAfter( root.node_end(), Xml::Element("accuracy", "1e-5"));
                    // erase node for IKTrialSet
                    root.eraseNode(toolIter);

                    // Create an OpenSimDocument node and move root inside it
                    Xml::Document newDocument;
                    Xml::Element docElement= newDocument.getRootElement();
                    docElement.setAttributeValue("Version", "20300");
                    docElement.setElementTag("OpenSimDocument");
                    // Copy all children of root to newRoot
                    docElement.insertNodeAfter(docElement.node_end(), doc.getRootElement().clone());
                    newDocument.writeToFile(newFileName);
                    setDocument(new XMLDocument(newFileName));
                    aNode = updDocument()->getRootDataElement();
                }
                else
                ;   // Something wrong! bail out
            }
        }
    }
    Object::updateFromXMLNode(aNode, versionNumber);
}
