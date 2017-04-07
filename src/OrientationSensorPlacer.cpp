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

#include "OpenSim/Tools/OrientationSensorPlacer.h"
#include "OpenSim/Common/OrientationSensorData.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Constant.h>
#include "OpenSim/Simulation/InverseKinematicsExtendedSolver.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Simulation/OrientationSensorsReference.h"
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Analyses/StatesReporter.h>
#include "OpenSim/Simulation/Model/ComponentSet.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationSensorPlacer::OrientationSensorPlacer() :
  _apply(_applyProp.getValueBool()),
  _oSensorFileName(_oSensorFileNameProp.getValueStr()),
  _timeRange(_timeRangeProp.getValueDblArray()),
  _ikExtendedTaskSetProp(PropertyObj("", IKExtendedTaskSet())),
  _ikExtendedTaskSet((IKExtendedTaskSet&)_ikExtendedTaskSetProp.getValueObj()),
  _coordinateFileName(_coordinateFileNameProp.getValueStr()),
  _outputModelFileName(_outputModelFileNameProp.getValueStr()),
  _outputOSensorFileName(_outputOSensorFileNameProp.getValueStr()),
  _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
  _maxOSensorMovement(_maxOSensorMovementProp.getValueDbl())
{
  setNull();
  setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationSensorPlacer::~OrientationSensorPlacer()
{
  //delete _ikTrial;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacer MarkerPlacer to be copied.
 */
OrientationSensorPlacer::OrientationSensorPlacer(const OrientationSensorPlacer &anOrientationSensorPlacer) :
  Object(anOrientationSensorPlacer),
  _apply(_applyProp.getValueBool()),
  _oSensorFileName(_oSensorFileNameProp.getValueStr()),
  _timeRange(_timeRangeProp.getValueDblArray()),
  _ikExtendedTaskSetProp(PropertyObj("", IKExtendedTaskSet())),
  _ikExtendedTaskSet((IKExtendedTaskSet&)_ikExtendedTaskSetProp.getValueObj()),
  _coordinateFileName(_coordinateFileNameProp.getValueStr()),
  _outputModelFileName(_outputModelFileNameProp.getValueStr()),
  _outputOSensorFileName(_outputOSensorFileNameProp.getValueStr()),
  _outputMotionFileName(_outputMotionFileNameProp.getValueStr()),
  _maxOSensorMovement(_maxOSensorMovementProp.getValueDbl())
{
  setNull();
  setupProperties();
  copyData(anOrientationSensorPlacer);
}

//_____________________________________________________________________________
/**
 * Copy data members from one OrientationSensorPlacer to another.
 *
 * @param anOrientationSensorPlacer OrientationSensorPlacer to be copied.
 */
void OrientationSensorPlacer::copyData(const OrientationSensorPlacer &anOrientationSensorPlacer)
{
  _apply = anOrientationSensorPlacer._apply;
  _oSensorFileName = anOrientationSensorPlacer._oSensorFileName;
  _timeRange = anOrientationSensorPlacer._timeRange;
  _ikExtendedTaskSet = anOrientationSensorPlacer._ikExtendedTaskSet;
  _coordinateFileName = anOrientationSensorPlacer._coordinateFileName;
  _outputModelFileName = anOrientationSensorPlacer._outputModelFileName;
  _outputOSensorFileName = anOrientationSensorPlacer._outputOSensorFileName;
  _outputMotionFileName = anOrientationSensorPlacer._outputMotionFileName;
  _maxOSensorMovement = anOrientationSensorPlacer._maxOSensorMovement;
  _printResultFiles = anOrientationSensorPlacer._printResultFiles;
  _outputStorage = NULL;
}

//_____________________________________________________________________________
/**
 * Set the data members of this OrientationSensorPlacer to their null values.
 */
void OrientationSensorPlacer::setNull()
{
  _apply = true;
  _coordinateFileName = "";

  _printResultFiles = true;
  _moveModelOSensors = true;
  _outputStorage = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void OrientationSensorPlacer::setupProperties()
{
  _applyProp.setComment("Whether or not to use the marker placer during scale");
  _applyProp.setName("apply");
  _propertySet.append(&_applyProp);

  _ikExtendedTaskSetProp.setComment("Task set used to specify weights used in the IK computation of the static pose.");
  _ikExtendedTaskSetProp.setName("IKExtendedTaskSet");
  _propertySet.append(&_ikExtendedTaskSetProp);

  _oSensorFileNameProp.setComment("MOT file (.mot) containing the time history of experimental oSensor orientations "
    "(usually a static trial).");
  _oSensorFileNameProp.setName("orientationSensors_file");
  _propertySet.append(&_oSensorFileNameProp);

  _coordinateFileNameProp.setComment("Name of file containing the joint angles "
    "used to set the initial configuration of the model for the purpose of placing the oSensors. "
    "These coordinate values can also be included in the optimization problem used to place the oSensors. "
    "Before the model oSensors are placed, a single frame of an inverse kinematics (IK) problem is solved. "
    "The IK problem can be solved simply by matching oSensors positions, but if the model oSensors are not "
    "in the correct locations, the IK solution will not be very good and neither will oSensors placement. "
    "Alternatively, coordinate values (specified in this file) can be specified and used to influence the IK solution. "
    "This is valuable particularly if you have high confidence in the coordinate values. "
    "For example, you know for the static trial the subject was standing will all joint angles close to zero. "
    "If the coordinate set (see the CoordinateSet property) contains non-zero weights for coordinates, "
    "the IK solution will try to match not only the oSensors orientations, but also the coordinates in this file. "
    "Least-squared error is used to solve the IK problem. ");
  _coordinateFileNameProp.setName("coordinate_file");
  _propertySet.append(&_coordinateFileNameProp);

  _timeRangeProp.setComment("Time range over which the oSensors orientations are averaged.");
  const double defaultTimeRange[] = { -1.0, -1.0 };
  _timeRangeProp.setName("time_range");
  _timeRangeProp.setValue(2, defaultTimeRange);
  _timeRangeProp.setAllowableListSize(2);
  _propertySet.append(&_timeRangeProp);

  _outputMotionFileNameProp.setComment("Name of the motion file (.mot) written after oSensors reorientations (optional).");
  _outputMotionFileNameProp.setName("output_motion_file");
  _propertySet.append(&_outputMotionFileNameProp);

  _outputModelFileNameProp.setComment("Output OpenSim model file (.osim) after scaling and maker placement.");
  _outputModelFileNameProp.setName("output_model_file");
  _propertySet.append(&_outputModelFileNameProp);

  _outputOSensorFileNameProp.setComment("Output oSensors set containing the new oSensor orientations after oSensors have been placed.");
  _outputOSensorFileNameProp.setName("output_oSensors_file");
  _propertySet.append(&_outputOSensorFileNameProp);

  _maxOSensorMovementProp.setComment("Maximum amount of movement allowed in oSensor data when averaging frames of the static trial. "
    "A negative value means there is not limit.");
  _maxOSensorMovementProp.setName("max_oSensors_movement");
  _maxOSensorMovementProp.setValue(-1.0); // units of this value are the units of the oSensor data in the static pose (usually rad)
  _propertySet.append(&_maxOSensorMovementProp);
}

//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
OrientationSensorPlacer& OrientationSensorPlacer::operator=(const OrientationSensorPlacer &anOrientationSensorPlacer)
{
  // BASE CLASS
  Object::operator=(anOrientationSensorPlacer);

  copyData(anOrientationSensorPlacer);

  return(*this);
}

//_____________________________________________________________________________
/**
 * This method implements the oSensors placing routine
 *
 * @param aModel the model to use for the marker placing process.
 * @return Whether the oSensor orienting process was successful or not.
 */
bool OrientationSensorPlacer::processModel(SimTK::State& s, Model* aModel, const string& aPathToSubject)
{
  if (!getApply())
    return false;
  cout << endl << "Step 3: Placing oSensor on model" << endl;

  // Load the static pose oSensor file, and average all the frames in the user-specified time range.
  OrientationSensorData staticPose(aPathToSubject + _oSensorFileName);
  if (_timeRange.getSize() < 2)
    throw Exception("OrientationSensorPlacer::processModel, time_range is unspecified.");
  staticPose.averageFrames(_maxOSensorMovement, _timeRange[0], _timeRange[1]);

  // Create references and WeightSets needed to initialize InverseKinemaicsSolver
  Set<OrientationSensorWeight> oSensorWeightSet;
  _ikExtendedTaskSet.createOrientationSensorsWeightSet(oSensorWeightSet); // order in tasks file
  OrientationSensorsReference oSensorsReference(staticPose, &oSensorWeightSet);
  SimTK::Array_<CoordinateReference> coordinateReferences;

  // Load the coordinate data
  // create CoordinateReferences for Coordinate Tasks
  FunctionSet *coordFunctions = NULL;
  bool haveCoordinateFile = false;
  if (_coordinateFileName != "" && _coordinateFileName != "Unassigned") {
    Storage coordinateValues(aPathToSubject + _coordinateFileName);
    // TODO: this check was missing, is it an OpenSim bug?
    if (coordinateValues.isInDegrees())
      aModel->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
    haveCoordinateFile = true;
    coordFunctions = new GCVSplineSet(5, &coordinateValues);
  }

  int index = 0;
  for (int i = 0; i < _ikExtendedTaskSet.getSize(); i++) {
    IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&_ikExtendedTaskSet[i]);
    if (coordTask && coordTask->getApply()) {
      CoordinateReference *coordRef = NULL;
      if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
        index = coordFunctions->getIndex(coordTask->getName(), index);
        if (index >= 0) {
          coordRef = new CoordinateReference(coordTask->getName(), coordFunctions->get(index));
        }
      }
      else if ((coordTask->getValueType() == IKCoordinateTask::ManualValue)) {
        Constant reference(Constant(coordTask->getValue()));
        coordRef = new CoordinateReference(coordTask->getName(), reference);
      }
      else { // assume it should be held at its current/default value
        double value = aModel->getCoordinateSet().get(coordTask->getName()).getValue(s);
        Constant reference = Constant(value);
        coordRef = new CoordinateReference(coordTask->getName(), reference);
      }

      if (coordRef == NULL)
        throw Exception("OrientationSensorPlacer: value for coordinate " + coordTask->getName() + " not found.");

      // We have a valid coordinate reference so now set its weight according to the task
      coordRef->setWeight(coordTask->getWeight());
      coordinateReferences.push_back(*coordRef);
    }
  }
  double constraintWeight = std::numeric_limits<SimTK::Real>::infinity();

  InverseKinematicsExtendedSolver* ikSol = new InverseKinematicsExtendedSolver(*aModel, oSensorsReference, coordinateReferences, constraintWeight);
  ikSol->assemble(s);

  // Call realize Position so that the transforms are updated and oSensors can be moved correctly
  aModel->getMultibodySystem().realize(s, SimTK::Stage::Position);

  // Report oSensor errors to assess the quality
  int nos = oSensorWeightSet.getSize();
  SimTK::Array_<double> oSensorErrors(nos, 0.0);
  double totalError = 0.0;
  double maxOSensorError = 0.0;
  double squaredSum = 0.0;
  int worstOSensor = -1;

  ikSol->computeCurrentOSensorErrors(oSensorErrors);

  for (int j = 0; j < nos; ++j) {
    totalError +=oSensorErrors[j];
    squaredSum += SimTK::square(oSensorErrors[j]);
    if (oSensorErrors[j] > maxOSensorError) {
      maxOSensorError = oSensorErrors[j];
      worstOSensor = j;
    }
  }
  double rmsError = sqrt(squaredSum / nos);

  cout << "OSensors tracking - Frame at (t=" << s.getTime() << "):\t";
  cout << "total absolute angular error = " << SimTK::convertRadiansToDegrees(totalError) << " degs";
  cout << ", RMS angular error = " << SimTK::convertRadiansToDegrees(rmsError) << " degs";
  cout << ", max absolute angular error = " << SimTK::convertRadiansToDegrees(maxOSensorError) << " degs ( " << ikSol->getOSensorNameForIndex(worstOSensor) << " )" << endl;

  /* Now move the non-fixed oSensor on the model so that they are coincident
   * with the measured oSensor in the static pose. The model is already in
   * the proper configuration so the coordinates do not need to be changed.
   */
  if (_moveModelOSensors) moveModelOSensorsToPose(s, *aModel, staticPose);

  if (_outputStorage != NULL) {
    delete _outputStorage;
  }

  if (_printResultFiles) {
    if (!_outputModelFileNameProp.getValueIsDefault())
    {
      aModel->print(aPathToSubject + _outputModelFileName);
      cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
    }

    // XML writing for orientationSensorsSet, still dirty since not directly supported by model
    if (!_outputOSensorFileNameProp.getValueIsDefault() && _outputOSensorFileName != "Unassigned") {
      OpenSim::ComponentSet &modelComponentSet = aModel->updMiscModelComponentSet();
      OpenSim::OrientationSensorSet modelOSensorSet;
      for (int i = 0; i < modelComponentSet.getSize(); ++i) {
        OpenSim::OrientationSensor* oSens = dynamic_cast<OpenSim::OrientationSensor*>(&modelComponentSet.get(i));
        if (oSens)
          modelOSensorSet.cloneAndAppend(*oSens);
      }
      modelOSensorSet.setName("OrientationSensorSet_from_model_"+aModel->getName());
      modelOSensorSet.print(aPathToSubject + _outputOSensorFileName);
      cout << "Wrote oSensor file " << _outputOSensorFileName << " from model " << aModel->getName() << endl;
    }

    // Write static pose (.mot)
    if (!_outputMotionFileNameProp.getValueIsDefault() && _outputMotionFileName != "Unassigned") {
      // Make a storage file containing the solved static pose state.
      // Requires a new state, previous one is messed up
      SimTK::State& st = aModel->initSystem();
      ikSol->assemble(st);
      aModel->getMultibodySystem().realize(st, SimTK::Stage::Position);

      Storage motionData;
      StatesReporter statesReporter(aModel);
      statesReporter.begin(st);

      _outputStorage = new Storage(statesReporter.updStatesStorage());
      _outputStorage->setName("static pose");

      _outputStorage->print(aPathToSubject + _outputMotionFileName,
        "w", "File generated from solving static pose for model " + aModel->getName());
    }
  }
  return true;
}

//_____________________________________________________________________________
/**
 * Set the local rotation offset of each oSensor so that in the model's
 * current pose the oSensor coincides with the oSensor's global orientation
 * in the passed-in oSensorData.
 *
 * @param aModel the model to use
 * @param aPose the static-pose oSensor cloud to get the oSensors orientations from
 */
void OrientationSensorPlacer::moveModelOSensorsToPose(SimTK::State& s, Model& aModel, OrientationSensorData& aPose)
{

  aPose.averageFrames(0.01);
  const OrientationSensorFrame &frame = aPose.getFrame(0);
  const SimbodyEngine& engine = aModel.getSimbodyEngine();

  OpenSim::ComponentSet &modelComponentSet = aModel.updMiscModelComponentSet();
  for (int i = 0; i < modelComponentSet.getSize(); ++i) {
    OpenSim::OrientationSensor* oSens = dynamic_cast<OpenSim::OrientationSensor*>(&modelComponentSet.get(i));
    if (oSens) {
      int index = aPose.getOrientationSensorIndex(oSens->getName());
      if (index >= 0) {
        // retrieve oSensor orientation from observation with respect to ground
        SimTK::Quaternion globalOSensor = frame.getOrientationSensor(index);
        if (!globalOSensor.isNaN()) {
          SimTK::Rotation R_GO(globalOSensor);
          // Does the same as engine.getDirectionCosines but without useless and bugged data format conversion
          SimTK::Rotation R_GB = aModel.getMatterSubsystem().getMobilizedBody(oSens->getBody().getIndex()).getBodyRotation(s);
          SimTK::Rotation R_BS_new = ~R_GB * R_GO;
          oSens->setRotationOffset(R_BS_new.convertRotationToBodyFixedXYZ());
        }
      }
      else {
        std::cout << "___WARNING___: oSensor " << oSens->getName() << " does not have valid coordinates in " << aPose.getFileName() << endl;
        std::cout << "               It will not be moved to match orientation in oSensor file." << endl;
      }
    }
  }
  cout << "Moved oSensors in model " << aModel.getName() << " to match orientations in oSensor file " << aPose.getFileName() << endl;
}

Storage *OrientationSensorPlacer::getOutputStorage()
{
  return _outputStorage; ;
}
