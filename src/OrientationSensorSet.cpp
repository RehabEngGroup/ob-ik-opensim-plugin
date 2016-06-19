/* -------------------------------------------------------------------------- *
 *                          OpenSim:  OrientationSensorSet.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
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

#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationSensorSet::~OrientationSensorSet(void)
{
}

//_____________________________________________________________________________
/**
 * Constructor of a OrientationSensorSet from a file.
 */
OrientationSensorSet::OrientationSensorSet(Model& aModel, const string& aOSensorsFileName) :
ModelComponentSet<OrientationSensor>(aModel, aOSensorsFileName, false)
{
    setNull();
    SimTK::Xml::Element e = updDocument()->getRootDataElement();
    updateFromXMLNode(e, getDocument()->getDocumentVersion());
}

//_____________________________________________________________________________
/**
 * Default constructor of a OrientationSensorSet.
 */
OrientationSensorSet::OrientationSensorSet() :
ModelComponentSet<OrientationSensor>()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a OrientationSensorSet.
 */
OrientationSensorSet::OrientationSensorSet(const OrientationSensorSet& aOrientationSensorSet):
ModelComponentSet<OrientationSensor>(aOrientationSensorSet)
{
    setNull();
    *this = aOrientationSensorSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this OrientationSensorSet to their null values.
 */
void OrientationSensorSet::setNull()
{
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
#ifndef SWIG
OrientationSensorSet& OrientationSensorSet::operator=(const OrientationSensorSet &aOrientationSensorSet)
{
    Set<OrientationSensor>::operator=(aOrientationSensorSet);
    return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get names of Orientation Sensors in the OrientationSensor set
 */
void OrientationSensorSet::getOSensorNames(Array<string>& aOSensorNamesArray)
{
    for (int i = 0; i < getSize(); i++)
    {
        OrientationSensor& nextOSensor = get(i);
        aOSensorNamesArray.append(nextOSensor.getName());
    }
}

//_____________________________________________________________________________
/**
 * Scale marker set by a set of scale factors
 // TODO: check if make sense to scale the orientation sensor location
 */
void OrientationSensorSet::scale(const ScaleSet& scaleSet)
{
    Vec3    scaleFactors(1.0);

    for (int i = 0; i < getSize(); i++)
    {
        OrientationSensor& nextOSensor = get(i);
        const string& refBodyName = nextOSensor.getBody().getName();
        //assert(refBodyName);
        bool found = false;
        for (int j = 0; j < scaleSet.getSize() && !found; j++)
        {
            Scale& nextScale = scaleSet.get(j);
            if (nextScale.getSegmentName() == refBodyName)
            {
                found = true;
                nextScale.getScaleFactors(scaleFactors);
                nextOSensor.scale(scaleFactors);
            }
        }
    }
}

//_____________________________________________________________________________
/**
 * Add name prefix.
 */
void OrientationSensorSet::addNamePrefix(const string& prefix)
{
    int i;

    // Cycle through set and add prefix
    for (i = 0; i < getSize(); i++)
        get(i).setName(prefix + get(i).getName());
}

//_____________________________________________________________________________
/**
 * Create a new orientation sensor and add it to the set.
 */
OrientationSensor* OrientationSensorSet::addOrientationSensor(const string& aName, const SimTK::Vec3& aPositionOffset, const SimTK::Vec3& aRotationOffset, OpenSim::Body& aBody)
{
    // If a OSensor by this name already exists, do nothing.
    if (contains(aName))
        return NULL;

    // Create a OSensor and add it to the set.
    OrientationSensor* m = new OrientationSensor();
    m->setName(aName);
    m->setPositionOffset(aPositionOffset);
    m->setRotationOffset(aRotationOffset);
    // Body will be based on this name when orientation sensor is connected to Model.
    m->setBodyName(aBody.getName());
    m->connectOSensorToModel(aBody.getModel());
    adoptAndAppend(m);

    return m;
}
