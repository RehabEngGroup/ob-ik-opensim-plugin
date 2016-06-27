/* -------------------------------------------------------------------------- *
 *                       OpenSim:  OrientationSensorsReference.cpp                       *
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

#include "OpenSim/Simulation/OrientationSensorsReference.h"
#include <OpenSim/Common/Units.h>

using namespace std;
using namespace SimTK;
using SimTK::Vec3;
namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the OrientationSensorsReference
 *
 * @param model to assemble
 */
OrientationSensorsReference::OrientationSensorsReference() : Reference_<SimTK::Quaternion>()
{
    setAuthors("Luca Tagliapietra, Elena Ceseracciu");
    constructProperties();
}

OrientationSensorsReference::OrientationSensorsReference(const std::string sensorFile) : Reference_<SimTK::Quaternion>()
{
    setAuthors("Luca Tagliapietra, Elena Ceseracciu");
    constructProperties();
    loadOrientationSensorsFile(sensorFile);
}

/**
 * Convenience constructor to be used for orientationSensor placement.
 *
 * @param aSensorData: orientationSensorData, assumed to be in the correct units already
 */
OrientationSensorsReference::OrientationSensorsReference(OrientationSensorData& aSensorData, const Set<OrientationSensorWeight>* aSensorWeightSet) : Reference_<SimTK::Quaternion>()
{
    if (aSensorWeightSet!=nullptr)
        setOrientationSensorWeightSet(*aSensorWeightSet);
    populateFromOrientationSensorData(aSensorData);
}

/** load the OSensor data for this OrientationSensorsReference from orientationSensorFile  */
void OrientationSensorsReference::loadOrientationSensorsFile(const std::string sensorFile)
{
    set_osensors_file(sensorFile);
    _orientationSensorData = new OrientationSensorData(get_osensors_file());

    populateFromOrientationSensorData(*_orientationSensorData);
}


/** A convenience method to populate OrientationSensorsReference from OrientationSensorData **/
void OrientationSensorsReference::populateFromOrientationSensorData(OrientationSensorData& aOSensorData)
{
    _orientationSensorData = &aOSensorData;
    const Array<std::string> &tempNames = aOSensorData.getOrientationSensorNames();
    int nos = tempNames.getSize();

    // empty any lingering names and weights
    _orientationSensorNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of oSensor in the file with default weightings
    _orientationSensorNames.assign(nos, "");
    _weights.assign(nos, get_default_weight());

    int index = 0;
    // Build flat lists (arrays) of oSensor names and weights in the same order as the oSensor data
    for(int i=0; i<nos; i++){
        const std::string &name = tempNames[i];
        _orientationSensorNames[i] = name;
        Set<OrientationSensorWeight> sensWeight=getOrientationSensorWeightSet();
        index=sensWeight.getIndex(name, index);
        //Assign user weighting for oSensors that are user listed in the input set
        if(index >= 0)
            _weights[i] = get_osensor_weights(index).getWeight();
    }

    if(_orientationSensorNames.size() != _weights.size())
        throw Exception("OrientationSensorsReference: Mismatch between the number of sensor names and weights. Verify that sensor names are unique.");
}

SimTK::Vec2 OrientationSensorsReference::getValidTimeRange() const
{
    return Vec2(_orientationSensorData->getStartFrameTime(), _orientationSensorData->getLastFrameTime());
}

// utility to define object properties including their tags, comments and
// default values.
void OrientationSensorsReference::constructProperties()
{
    constructProperty_osensors_file("");
    constructProperty_default_weight(1.0);
    constructProperty_osensor_weights();

}

/** get the names of the markers serving as references */
const SimTK::Array_<std::string>& OrientationSensorsReference::getNames() const
{
    return _orientationSensorNames;
}

/** get the values of the OrientationSensorsReference */
void  OrientationSensorsReference::getValues(const SimTK::State &s, SimTK::Array_<SimTK::Quaternion> &values) const
{
    double time =  s.getTime();

    int before=0, after=0;
    // get index for time
    _orientationSensorData->findFrameRange(time, time, before, after);
    if(before > after || after < 0)
        throw Exception("OrientationSensorsReference: No index corresponding to time of frame.");
    else if(after-before > 0){
        before = abs(_orientationSensorData->getFrame(before).getFrameTime() - time) < abs(_orientationSensorData->getFrame(after).getFrameTime() - time) ? before : after;
    }

    values = _orientationSensorData->getFrame(before).getOrientationSensors();
}

/** get the speed value of the OrientationSensorsReference */  //TODO
void OrientationSensorsReference::getSpeedValues(const SimTK::State &s, SimTK::Array_<Vec3> &speedValues) const
{
    throw Exception("OrientationSensorsReference: getSpeedValues not implemented.");
}

/** get the acceleration value of the OrientationSensorsReference */ //TODO
void OrientationSensorsReference::getAccelerationValues(const SimTK::State &s, SimTK::Array_<Vec3> &accValues) const
{
    throw Exception("OrientationSensorsReference: getAccelerationValues not implemented.");
}

/** get the weights of the oSensors */
void  OrientationSensorsReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
    weights = _weights;
}

void OrientationSensorsReference::setOrientationSensorWeightSet(const Set<OrientationSensorWeight> &sensorWeights)
{
    updProperty_osensor_weights().clear();
    for (int i = 0; i < sensorWeights.getSize(); i++)
        updProperty_osensor_weights().appendValue(sensorWeights[i]);
}

Set<OrientationSensorWeight> OrientationSensorsReference::getOrientationSensorWeightSet()
{
    Set<OrientationSensorWeight> sensWeight;
    for (int j = 0; j < getProperty_osensor_weights().size(); ++j){
      //std::cout << "inside osensor weights set at " << j;
      //std::cout << " sensor name is " << get_osensor_weights(j).getName() << " weight " << get_osensor_weights(j).getWeight() << std::endl;
        sensWeight.cloneAndAppend(get_osensor_weights(j)); //TODO check if value passed in is correct
    }
    return sensWeight;
}

} // end of namespace OpenSim