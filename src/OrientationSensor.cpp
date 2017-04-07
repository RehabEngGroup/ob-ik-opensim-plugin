/* -------------------------------------------------------------------------- *
 *         Orientation Based Inverse Kinematics: OrientationSensor.cpp        *
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
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

// TODO: Fix visualization functions and variables
Geometry *OrientationSensor::_defaultGeometry = new AnalyticEllipsoid(0.04, 0.02, 0.01);
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
OrientationSensor::OrientationSensor() :
ModelComponent()
{
    setNull();
    constructProperties();
    _displayer.setOwner(this);
   // _displayer.addGeometry(_defaultGeometry);
   // _displayer.setShowAxes(true);
}

//_____________________________________________________________________________
/**
* Destructor.
*/
OrientationSensor::~OrientationSensor()
{
}

void OrientationSensor::registerTypes()
{
}

//_____________________________________________________________________________
/**
* Copy constructor.
*
* @param aOSensor oSensor to be copied.
*/
OrientationSensor::OrientationSensor(const OrientationSensor &aOSensor) :
ModelComponent(aOSensor)
{
    copyData(aOSensor);
    _displayer.setOwner(this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Copy data members from one oSensor to another.
*
* @param aOSensor oSensor to be copied.
*/
void OrientationSensor::copyData(const OrientationSensor &aOSensor)
{
    _body = aOSensor._body; //TODO check if correct
    _displayer = aOSensor._displayer;
    _virtual = aOSensor._virtual;
    copyProperty_body(aOSensor);
    copyProperty_fixed(aOSensor);
    copyProperty_position_offset(aOSensor);
    copyProperty_rotation_offset(aOSensor);
}

//_____________________________________________________________________________
/**
* Set the data members of this oSensor to their null values.
*/
void OrientationSensor::setNull()
{
    setVirtual(true);
    _body = 0;
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void OrientationSensor::constructProperties()
{
    constructProperty_body("");
    constructProperty_position_offset(SimTK::Vec3(0));
    constructProperty_fixed(false);
    constructProperty_rotation_offset(SimTK::Vec3(0));

}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aEngine dynamics engine containing this Marker.
*/
void OrientationSensor::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
    _model = &aModel;

    if (get_body() != "")
    {
        OpenSim::Body *oldBody = _body;
        try {
            _body = &const_cast<Model*>(&aModel)->updBodySet().get(get_body());
        }
        catch (const Exception&) {
            string errorMessage = "Error: Body " + get_body() + " referenced in oSensor " + getName() +
                " does not exist in model " + aModel.getName();
            throw Exception(errorMessage);
        }
        //TODO ELENA: adapt this to new properties
        // If marker jumped between bodies we need to fix display stuff
        //if (oldBody && oldBody->getName() != _body->getName()){
        //    oldBody->updDisplayer()->removeDependent(&_displayer);
        //}
        // Todo_AYMAN: make code below safe to call multiple times (since this
        // method may be called multiple times for a marker). Should also try
        // to handle case where a marker is deleted (e.g. in
        // SimbodyEngine::updateMarkerSet) because then may end up with
        // stale pointers.
        VisibleObject* ownerBodyDisplayer;
        if (_body && (ownerBodyDisplayer = _body->updDisplayer())){
            if (!ownerBodyDisplayer->hasDependent(&_displayer)){	// Only if first time to be encountered
                ownerBodyDisplayer->addDependent(&_displayer);
            }
        }
        _displayer.setOwner(this);
        updateGeometry();

    }
    else
    {
        string errorMessage = "Error: No body name specified for oSensor " + getName() + " in model " +
            aModel.getName();
        throw Exception(errorMessage);
    }
}

//_____________________________________________________________________________
/**
* Remove self from the list of displayable objects and free resources
*/
void OrientationSensor::removeSelfFromDisplay()
{
    VisibleObject* ownerBodyDisplayer;
    if (_body && (ownerBodyDisplayer = _body->updDisplayer())){
        if (ownerBodyDisplayer->hasDependent(&_displayer)){
            ownerBodyDisplayer->removeDependent(&_displayer);
        }
    }
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
OrientationSensor& OrientationSensor::operator=(const OrientationSensor &aMarker)
{
    // BASE CLASS
    Object::operator=(aMarker);

    copyData(aMarker);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
* Update an existing marker with parameter values from a
* new one, but only for the parameters that were explicitly
* specified in the XML node.
*
* @param aMarker marker to update from
*/
void OrientationSensor::updateFromOSensor(const OrientationSensor &aOSensor)
{
    if (!aOSensor.getProperty_position_offset().getValueIsDefault() )
    {
        const Vec3& off = aOSensor.get_position_offset();
        set_position_offset(off);
    }
    if (!aOSensor.getProperty_rotation_offset().getValueIsDefault())
    {
        const Vec3& off = aOSensor.get_rotation_offset();
        set_rotation_offset(off);
    }

    if (!aOSensor.getProperty_fixed().getValueIsDefault())
    {
        set_fixed(aOSensor.get_fixed());    }

    if (!aOSensor.getProperty_body().getValueIsDefault())
    {
        set_body(aOSensor.get_body());
    }
}



//_____________________________________________________________________________
/**
* Change the body that this marker is attached to. It assumes that the body is
* already set, so that connectMarkerToModel() needs to be called to update
* dependent information.
*
* @param aBody Reference to the body.
*/
void OrientationSensor::changeBody(OpenSim::Body& aBody)
{

    if (&aBody == _body)
        return;

    set_body(aBody.getName());
    connectToModel(aBody.updModel());
}

//_____________________________________________________________________________
/**
* Change the body that this marker is attached to. It assumes that the body is
* already set, so that connectMarkerToModel() needs to be called to update
* dependent information.
*
* @param s State.
* @param aBody Reference to the body.
*/
void OrientationSensor::changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody)
{

    if (&aBody == _body || !_body)
        return;

    // Preserve location means to switch bodies without changing
    // the location of the marker in the inertial reference frame.
    SimTK::Vec3 newOffset;
    aBody.getModel().getSimbodyEngine().transformPosition(s, *_body, get_position_offset(), aBody, newOffset);
    set_position_offset(newOffset);
    //TODO handle rotation
    //aBody.getModel().getSimbodyEngine().get
    set_body(aBody.getName());
    connectToModel(aBody.updModel());
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
* Scale the marker.
*
* @param aScaleFactors XYZ scale factors.
*/
// TODO: Check if make sense the scaling
void OrientationSensor::scale(const SimTK::Vec3& aScaleFactors)
{
    SimTK::Vec3 off = get_position_offset();
    for (int i = 0; i < 3; i++)
        off[i] *= aScaleFactors[i];
    set_position_offset(off);
}

void OrientationSensor::setPositionOffset(const SimTK::Vec3& offset)
{
    set_position_offset(offset);
    updateGeometry();
}

void OrientationSensor::setRotationOffset(const SimTK::Vec3& offset)
{
    set_rotation_offset(offset);
    updateGeometry();
}

void OrientationSensor::setBodyName(const std::string aBodyName)
{
    set_body(aBodyName);
}

SimTK::Vec3 OrientationSensor::getPositionOffset(){
    return get_position_offset();
}

SimTK::Vec3 OrientationSensor::getRotationOffset(){
    return get_rotation_offset();
}

//_____________________________________________________________________________
/**
* Update the geometry to correspond to position changes
*/
void OrientationSensor::updateGeometry()
{
    Transform position;
    position.setP(get_position_offset());
    SimTK::Rotation rot;
    rot.setRotationToBodyFixedXYZ(get_rotation_offset());
    position.updR() = SimTK::Rotation(rot);
    updDisplayer()->setTransform(position);

}

void OrientationSensor::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    Super::generateDecorations(fixed, hints, state, appendToThis);
    if (fixed == false) {
        const Vec3 pink(1, .6, .8);
        appendToThis.push_back(SimTK::DecorativeBrick(SimTK::Vec3(0.04, 0.02, 0.01)).setBodyId(this->getBody().getIndex()).setColor(pink).setTransform(getDisplayer()->getTransform()));
    }
}
