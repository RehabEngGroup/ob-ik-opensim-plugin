
//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Simulation/OrientationSensor.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

Geometry *OrientationSensor::_defaultGeometry = new AnalyticCylinder(0.01, 0.03);
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
OrientationSensor::OrientationSensor() :
Object()
{
    setNull();
    constructProperties();
    _displayer.setOwner(this);
}

//_____________________________________________________________________________
/**
* Destructor.
*/
OrientationSensor::~OrientationSensor()
{
}

//_____________________________________________________________________________
/**
* Copy constructor.
*
* @param aMarker Marker to be copied.
*/
OrientationSensor::OrientationSensor(const OrientationSensor &aMarker) :
Object(aMarker)
{
    setNull();
    constructProperties();
    copyData(aMarker);
    _displayer.setOwner(this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Copy data members from one Marker to another.
*
* @param aMarker Marker to be copied.
*/
void OrientationSensor::copyData(const OrientationSensor &aMarker)
{
    _body = NULL;
    _displayer = aMarker._displayer;
    _virtual = aMarker._virtual;
}

//_____________________________________________________________________________
/**
* Set the data members of this Marker to their null values.
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
    constructProperty_offset(SimTK::Vec3());
    constructProperty_fixed(false);
    constructProperty_rotation(SimTK::Vec3());

}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aEngine dynamics engine containing this Marker.
*/
void OrientationSensor::connectOSensorToModel(const Model& aModel)
{

    _model = &aModel;

    if (get_body() != "")
    {
        OpenSim::Body *oldBody = _body;
        try {
            _body = &const_cast<Model*>(&aModel)->updBodySet().get(get_body());
        }
        catch (const Exception&) {
            string errorMessage = "Error: Body " + get_body() + " referenced in marker " + getName() +
                " does not exist in model " + aModel.getName();
            throw Exception(errorMessage);
        }
        // If marker jumped between bodies we need to fix display stuff
        if (oldBody && oldBody->getName() != _body->getName()){
            oldBody->updDisplayer()->removeDependent(&_displayer);
        }
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
        string errorMessage = "Error: No body name specified for marker " + getName() + " in model " +
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
    if (!aOSensor.getProperty_offset().getValueIsDefault() )
    {
        const Vec3& off = aOSensor.get_offset();
        set_offset(off);
    }
    if (!aOSensor.getProperty_rotation().getValueIsDefault())
    {
        const Vec3& off = aOSensor.get_rotation();
        set_rotation(off);
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
    connectOSensorToModel(aBody.getModel());
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
    aBody.getModel().getSimbodyEngine().transformPosition(s, *_body, get_offset(), aBody, newOffset);
    set_offset(newOffset);
    //TODO handle rotation
    set_body(aBody.getName());
    connectOSensorToModel(aBody.getModel());
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
void OrientationSensor::scale(const SimTK::Vec3& aScaleFactors)
{
    SimTK::Vec3 off = get_offset();
    for (int i = 0; i < 3; i++)
        off[i] *= aScaleFactors[i];
    set_offset(off);
}

//_____________________________________________________________________________
/**
* Update the geometry to correspond to position changes
*/
void OrientationSensor::updateGeometry()
{
    Transform position;
    position.setP(get_offset());
    SimTK::Rotation rot;
    rot.setRotationToBodyFixedXYZ(get_rotation());
    position.updR() = SimTK::Rotation(rot);
    updDisplayer()->setTransform(position);

}
