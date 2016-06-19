/* -------------------------------------------------------------------------- *
 *                   OpenSim:  InverseKinematicsExtendedSolver.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "OpenSim/Simulation/InverseKinematicsExtendedSolver.h"
#include "OpenSim/Simulation/CoordinateReference.h"
#include "OpenSim/Simulation/MarkersReference.h"
#include "OpenSim/Simulation/OrientationSensorsReference.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/MarkerSet.h"
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the InverseKinematicsExtendedSolver
 *
 * @param model to assemble
 */
InverseKinematicsExtendedSolver::InverseKinematicsExtendedSolver(const Model &model, MarkersReference &markersReference,
                            OrientationSensorsReference &orientationSensorsReference,
                            SimTK::Array_<CoordinateReference> &coordinateReferences,
                            double constraintWeight) : AssemblySolver(model, coordinateReferences, constraintWeight),
                            _markersReference(markersReference),
                            _orientationSensorsReference(orientationSensorsReference)
{
    setAuthors("Luca Tagliapietra, Elena Ceseracciu");

    // Base AssemblySolver takes care of creating the underlying _assembler and setting up CoordinateReferences;
    _markerAssemblyCondition = NULL;
    _orientationSensorAssemblyCondition = NULL;

    // Do some consistency checking for markers and orientation sensors
    const MarkerSet &modelMarkerSet = getModel().getMarkerSet();
    const ComponentSet &modelComponentSet = getModel().getMiscModelComponentSet();
    OrientationSensorSet modelOSensorSet;
    for (int i = 0; i < modelComponentSet.getSize(); ++i) {
        OrientationSensor* oSens = dynamic_cast<OrientationSensor*>(&modelComponentSet.get(i));
        if (oSens != nullptr)
            modelOSensorSet.cloneAndAppend(*oSens); //TODO check if adoptAndAppend is appropriate
    }
    if(modelMarkerSet.getSize() < 1){
        std::cout << "InverseKinematicsExtendedSolver: Model has no markers!"  << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: Model has no markers!");
    }
    if (modelOSensorSet.getSize() < 1){
        std::cout << "InverseKinematicsExtendedSolver: Model has no orientation sensors!" << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: Model has no orientation sensors!");
    }

    const SimTK::Array_<std::string> &markerNames = _markersReference.getNames(); // size and content as in trc file

    if(markerNames.size() < 1){
        std::cout << "InverseKinematicsExtendedSolver: No markers available from data provided."  << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: No markers available from data provided.");
    }

    const SimTK::Array_<std::string> &oSensorNames = _orientationSensorsReference.getNames();

    if (oSensorNames.size() < 1) {
        std::cout << "InverseKinematicsExtendedSolver: No orientation sensors available from data provided." << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: No orientation sensors available from data provided.");
    }

    int markerIndex=0, markerCnt=0;
    for(unsigned int i=0; i < markerNames.size(); i++) {
        // Check if we have this marker in the model, else ignore it
        markerIndex = modelMarkerSet.getIndex(markerNames[i], markerIndex);
        if (markerIndex >= 0) //found corresponding model
            markerCnt++;
    }

    if (markerCnt < 1){
        std::cout <<"InverseKinematicsExtendedSolver: Marker data does not correspond to any model markers." << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: Marker data does not correspond to any model markers.");
    }
    if (markerCnt < 4)
        cout << "WARNING: InverseKinematicsExtendedSolver found only " << markerCnt << " markers to track." << endl;

    int oSensorIndex = 0, oSensorCnt = 0;
    for (unsigned int i = 0; i < oSensorNames.size(); i++) {
        // Check if we have this marker in the model, else ignore it
        oSensorIndex = modelOSensorSet.getIndex(oSensorNames[i], oSensorIndex);
        if (oSensorIndex >= 0) //found corresponding model
            oSensorCnt++;
    }

    if (oSensorCnt < 1){
        std::cout << "InverseKinematicsExtendedSolver: Orientation sensor data does not correspond to any model sensors." << std::endl;
        throw Exception("InverseKinematicsExtendedSolver: Orientation sensor data does not correspond to any model sensors.");
    }
    if (oSensorCnt < 5)
        cout << "WARNING: InverseKinematicsExtendedSolver found only " << oSensorCnt << " orientation sensors to track." << endl;

}

/** Change the weighting of a marker to take effect when assemble or track is called next.
    Update a marker's weight by name. */
void InverseKinematicsExtendedSolver::updateMarkerWeight(const std::string &markerName, double value)
{
    const Array_<std::string> &names = _markersReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    updateMarkerWeight(index, value);
}

void InverseKinematicsExtendedSolver::updateOSensorWeight(const std::string &sensorName, double value)
{
    const Array_<std::string> &names = _orientationSensorsReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), sensorName);
    int index = (int)std::distance(names.begin(), p);
    updateOSensorWeight(index, value);
}

/** Update a marker's weight by its index. */
void InverseKinematicsExtendedSolver::updateMarkerWeight(int markerIndex, double value)
{
    if(markerIndex >=0 && markerIndex < _markersReference.updMarkerWeightSet().getSize()){
        _markersReference.updMarkerWeightSet()[markerIndex].setWeight(value);
        _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(markerIndex), value);
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::updateMarkerWeight: invalid markerIndex.");
}

void InverseKinematicsExtendedSolver::updateOSensorWeight(int sensorIndex, double value)
{
    if (sensorIndex >= 0 && sensorIndex < _orientationSensorsReference.getNumRefs()){
        _orientationSensorsReference.upd_osensor_weights(sensorIndex).setWeight(value);
       // _orientationSensorsReference.updProperty_osensor_weights().updValue()[sensorIndex].setWeight(value);
       // _markersReference.updMarkerWeightSet()[markerIndex].setWeight(value);
        _orientationSensorAssemblyCondition->changeOSensorWeight(SimTK::OrientationSensors::OSensorIx(sensorIndex), value);
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::updateOrientationSensorWeight: invalid oSensorIndex.");
}

/** Update all markers weights by order in the markersReference passed in to
    construct the solver. */
void InverseKinematicsExtendedSolver::updateMarkerWeights(const SimTK::Array_<double> &weights)
{
    if(static_cast<unsigned>(_markersReference.updMarkerWeightSet().getSize())
       == weights.size()){
        for(unsigned int i=0; i<weights.size(); i++){
            _markersReference.updMarkerWeightSet()[i].setWeight(weights[i]);
            _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(i), weights[i]);
        }
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::updateMarkerWeights: invalid size of weights.");
}

void InverseKinematicsExtendedSolver::updateOSensorWeights(const SimTK::Array_<double> &weights)
{
    if (static_cast<unsigned>(_orientationSensorsReference.getNumRefs()) == weights.size()){
        for (unsigned int i = 0; i<weights.size(); i++){
            _orientationSensorsReference.upd_osensor_weights(i).setWeight(weights[i]);
            _orientationSensorAssemblyCondition->changeOSensorWeight(SimTK::OrientationSensors::OSensorIx(i), weights[i]);
        }
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::updateOrientationSensorWeights: invalid size of weights.");
}

/** Compute and return the spatial location of a marker in ground. */
SimTK::Vec3 InverseKinematicsExtendedSolver::computeCurrentMarkerLocation(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerLocation(index);
}

SimTK::Quaternion InverseKinematicsExtendedSolver::computeCurrentOSensorOrientation(const std::string &sensorName)
{
    const Array_<std::string> &names = _orientationSensorsReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), sensorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentOSensorOrientation(index);
}

SimTK::Vec3 InverseKinematicsExtendedSolver::computeCurrentMarkerLocation(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::computeCurrentMarkerLocation: invalid markerIndex.");
}

SimTK::Quaternion InverseKinematicsExtendedSolver::computeCurrentOSensorOrientation(int sensorIndex)
{
    if (sensorIndex >= 0 && sensorIndex < _orientationSensorAssemblyCondition->getNumOSensors()){
        return _orientationSensorAssemblyCondition->findCurrentOSensorOrientation(SimTK::OrientationSensors::OSensorIx(sensorIndex)).convertRotationToQuaternion();
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::computeCurrentMarkerLocation: invalid markerIndex.");
}

/** Compute and return the spatial locations of all markers in ground. */
void InverseKinematicsExtendedSolver::computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations)
{
    markerLocations.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerLocations.size(); i++)
        markerLocations[i] = _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(i));
}

void InverseKinematicsExtendedSolver::computeCurrentOSensorOrientations(SimTK::Array_<SimTK::Quaternion> &sensorOrientations)
{
    sensorOrientations.resize(_orientationSensorAssemblyCondition->getNumOSensors());
    for (unsigned int i = 0; i<sensorOrientations.size(); i++)
        sensorOrientations[i] = _orientationSensorAssemblyCondition->findCurrentOSensorOrientation(SimTK::OrientationSensors::OSensorIx(i)).convertRotationToQuaternion();
}


/** Compute and return the distance error between model marker and observation. */
double InverseKinematicsExtendedSolver::computeCurrentMarkerError(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerError(index);
}

double InverseKinematicsExtendedSolver::computeCurrentOSensorError(const std::string &oSensorName)
{
    const Array_<std::string> &names = _orientationSensorsReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), oSensorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentOSensorError(index);
}

double InverseKinematicsExtendedSolver::computeCurrentMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::computeCurrentMarkerError: invalid markerIndex.");
}

double InverseKinematicsExtendedSolver::computeCurrentOSensorError(int sensorIndex)
{
    if (sensorIndex >= 0 && sensorIndex < _orientationSensorAssemblyCondition->getNumOSensors()){
        return _orientationSensorAssemblyCondition->findCurrentOSensorError(SimTK::OrientationSensors::OSensorIx(sensorIndex));
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::computeCurrentOSensorError: invalid sensorIndex.");
}

/** Compute and return the distance errors between all model markers and their observations. */
void InverseKinematicsExtendedSolver::computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(i));
}

void InverseKinematicsExtendedSolver::computeCurrentOSensorErrors(SimTK::Array_<double> &oSensorErrors)
{
    oSensorErrors.resize(_orientationSensorAssemblyCondition->getNumOSensors());
    for (unsigned int i = 0; i<oSensorErrors.size(); i++)
        oSensorErrors[i] = _orientationSensorAssemblyCondition->findCurrentOSensorError(SimTK::OrientationSensors::OSensorIx(i));
}

/** Compute and return the squared-distance error between model marker and observation. */
double InverseKinematicsExtendedSolver::computeCurrentSquaredMarkerError(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference.getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentSquaredMarkerError(index);
}

//double InverseKinematicsExtendedSolver::computeCurrentSquaredOSensorError(const std::string &sensorName)
//{
//    const Array_<std::string> &names = _orientationSensorsReference.getNames();
//    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), sensorName);
//    int index = (int)std::distance(names.begin(), p);
//    return computeCurrentSquaredOSensorError(index);
//}

double InverseKinematicsExtendedSolver::computeCurrentSquaredMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsExtendedSolver::computeCurrentMarkerSquaredError: invalid markerIndex.");
}

//double InverseKinematicsExtendedSolver::computeCurrentSquaredOSensorError(int sensorIndex)
//{
//    if (sensorIndex >= 0 && sensorIndex < _orientationSensorAssemblyCondition->getNumOSensors()){
//        return _orientationSensorAssemblyCondition->findCurrentOSensorErrorSquared(SimTK::OrientationSensors::OSensorIx(sensorIndex));
//    }
//    else
//        throw Exception("InverseKinematicsExtendedSolver::computeCurrentMarkerSquaredError: invalid markerIndex.");
//}

/** Compute and return the distance errors between all model marker and observations. */
void InverseKinematicsExtendedSolver::computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(i));
}

/** Marker errors are reported in order different from tasks file or model, find name corresponding to passed in index  */
std::string InverseKinematicsExtendedSolver::getMarkerNameForIndex(int markerIndex) const
{
    return _markerAssemblyCondition->getMarkerName(SimTK::Markers::MarkerIx(markerIndex));
}

std::string InverseKinematicsExtendedSolver::getOSensorNameForIndex(int sensorIndex) const
{
    return _orientationSensorAssemblyCondition->getOSensorName(SimTK::OrientationSensors::OSensorIx(sensorIndex));
}

/** Internal method to convert the MarkerReferences into additional goals of the
    of the base assembly solver, that is going to do the assembly.  */
void InverseKinematicsExtendedSolver::setupGoals(SimTK::State &s)
{
  // Setup coordinates performed by the base class
  AssemblySolver::setupGoals(s);

  _markerAssemblyCondition = new SimTK::Markers();

  // Setup markers goals
  // Get lists of all markers by names and corresponding weights from the MarkersReference
  const SimTK::Array_<SimTK::String> &markerNames = _markersReference.getNames();
  SimTK::Array_<double> markerWeights;
  _markersReference.getWeights(s, markerWeights);
  // get markers defined by the model
  const MarkerSet &modelMarkerSet = getModel().getMarkerSet();

  // get markers with specified tasks/weights
  const Set<MarkerWeight>& mwSet = _markersReference.updMarkerWeightSet();

  int index = -1;
  int wIndex = -1;
  //Loop through all markers in the reference
  for (unsigned int i = 0; i < markerNames.size(); ++i) {
    // Check if we have this marker in the model, else ignore it
    index = modelMarkerSet.getIndex(markerNames[i], index);
    wIndex = mwSet.getIndex(markerNames[i], wIndex);
    if ((index >= 0) && (wIndex >= 0)) {
      Marker &marker = modelMarkerSet[index];
      const SimTK::MobilizedBody& mobod =
        getModel().getMatterSubsystem().getMobilizedBody(marker.getBody().getIndex());
      _markerAssemblyCondition->
        addMarker(marker.getName(), mobod, marker.getOffset(), markerWeights[i]);

      //cout << "IKSolver Marker: " << markerNames[i] << " " << marker.getName() << "  weight: " << markerWeights[i] << endl;
    }
  }
  if (_markerAssemblyCondition->getNumMarkers() > 0){
    _assembler->adoptAssemblyGoal(_markerAssemblyCondition);
    _markerAssemblyCondition->defineObservationOrder(markerNames);
  }
      // Add marker goal to the ik objective
    // lock-in the order that the observations (markers) are in and this cannot change from frame to frame
    // and we can use an array of just the data for updating

    // ORIENTATION SENSORS
    _orientationSensorAssemblyCondition = new SimTK::OrientationSensors();
    // Setup orientation goals
    // Get lists of all orientation sensors by names and corresponding weights from the OrientationSensorsReference
    const SimTK::Array_<SimTK::String> &oSensorNames = _orientationSensorsReference.getNames();
    SimTK::Array_<double> oSensorWeights;
    _orientationSensorsReference.getWeights(s, oSensorWeights);
    // get orientation sensors defined by the model
    const ComponentSet &modelComponentSet = getModel().getMiscModelComponentSet();
    OrientationSensorSet modelOSensorSet;
    for (int i = 0; i < modelComponentSet.getSize(); ++i) {
        OrientationSensor* oSens = dynamic_cast<OrientationSensor*>(&modelComponentSet.get(i));
        if (oSens != nullptr)
            modelOSensorSet.cloneAndAppend(*oSens); //TODO check if adoptAndAppend is appropriate
    }
    // get orientation sensors with specified tasks/weights
    const Set<OrientationSensorWeight>& owSet = _orientationSensorsReference.getOrientationSensorWeightSet();
    OpenSim::Array<std::string> names;
    owSet.getNames(names);
    for (int k = 0; k < names.size(); ++k)
        std::cout << "osensor names at " << k << " is " << names[k] << std::endl;

    index = -1;
    wIndex = -1;
    //Loop through all orientation sensors in the reference
    for (unsigned int i = 0; i < oSensorNames.size(); ++i){
        // Check if we have this marker in the model, else ignore it
        index = modelOSensorSet.getIndex(oSensorNames[i], index);
        wIndex = owSet.getIndex(oSensorNames[i], wIndex);
        std::cout << "index " << index << " windex " << wIndex << std::endl;
        if ((index >= 0) && (wIndex >= 0)){
            OrientationSensor &oSensor = modelOSensorSet[index];
            const SimTK::MobilizedBody& mobod =
                getModel().getMatterSubsystem().getMobilizedBody(oSensor.getBody().getIndex());
            SimTK::Rotation rot;
            rot.setRotationToBodyFixedXYZ(oSensor.getRotationOffset());
            _orientationSensorAssemblyCondition->
                addOSensor(oSensor.getName(), mobod, rot , oSensorWeights[i]);

            //cout << "IKSolver Marker: " << markerNames[i] << " " << marker.getName() << "  weight: " << markerWeights[i] << endl;
        }
    }
       // Add marker goal to the ik objective
    //_assembler->adoptAssemblyGoal(_markerAssemblyCondition);
    _assembler->adoptAssemblyGoal(_orientationSensorAssemblyCondition);

    // lock-in the order that the observations (markers) are in and this cannot change from frame to frame
    // and we can use an array of just the data for updating
  //  _markerAssemblyCondition->defineObservationOrder(markerNames);
    _orientationSensorAssemblyCondition->defineObservationOrder(oSensorNames);
    _assembler->initialize();
    SimTK::State p = s;
    updateGoals(p);
}

/** Internal method to update the time, reference values and/or their weights based
    on the state */
void InverseKinematicsExtendedSolver::updateGoals(const SimTK::State &p)
{
    SimTK::State s(p);
    // update coordinates performed by the base class
    AssemblySolver::updateGoals(s);

    if (_markerAssemblyCondition->isInAssembler()) {
      // specify the (initial) observations to be matched
      _markersReference.getValues(s, _markerValues);
      _markerAssemblyCondition->moveAllObservations(_markerValues);
    }
    _orientationSensorsReference.getValues(s, _orientationSensorValues);
    std::cout << _orientationSensorValues.at(0) << " orientation sensor values at 0 "<< std::endl;
    std::cout << _orientationSensorValues.at(1) << " orientation sensor values at 1 " << std::endl;
    _orientationSensorAssemblyCondition->moveAllObservations(_orientationSensorValues);
}

} // end of namespace OpenSim
