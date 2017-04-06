/* -------------------------------------------------------------------------- *
 *                            ScaleExtendedTool.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
 //=============================================================================
 /**
  * ScaleExtendedTool.cpp
  * The extended version of the original OpenSim ScaleTool class to allow orientation
  * sensor orientations adjustments on body segments during a static trial
  *
  * @author: Luca Tagliapietra <tagliapietra@gest.unipd.it>
  * @notes: beginning date Dec 2016
  */

  //=============================================================================
  // INCLUDES
  //=============================================================================
#include "OpenSim/Tools/OrientationSensorsPlacerTool.h"
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"
#include <OpenSim/Common/SimmIO.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "SimTKsimbody.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationSensorsPlacerTool::OrientationSensorsPlacerTool() :
  _mass(_massProp.getValueDbl()),
  _height(_heightProp.getValueDbl()),
  _age(_ageProp.getValueDbl()),
  _notes(_notesProp.getValueStr()),
  _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
  _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
  _oSensorPlacerProp(PropertyObj("", OrientationSensorPlacer())),
  _oSensorPlacer((OrientationSensorPlacer&)_oSensorPlacerProp.getValueObj())
{
  setNull();
  setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
OrientationSensorsPlacerTool::OrientationSensorsPlacerTool(const string &aFileName) :
  Object(aFileName, true),
  _mass(_massProp.getValueDbl()),
  _height(_heightProp.getValueDbl()),
  _age(_ageProp.getValueDbl()),
  _notes(_notesProp.getValueStr()),
  _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
  _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
  _oSensorPlacerProp(PropertyObj("", OrientationSensorPlacer())),
  _oSensorPlacer((OrientationSensorPlacer&)_oSensorPlacerProp.getValueObj())
{
  setNull();
  setupProperties();

  updateFromXMLDocument();

  _pathToSubject = IO::getParentDirectory(aFileName);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationSensorsPlacerTool::~OrientationSensorsPlacerTool()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSubject ScaleTool to be copied.
 */
OrientationSensorsPlacerTool::OrientationSensorsPlacerTool(const OrientationSensorsPlacerTool &aSubject) :
  Object(aSubject),
  _mass(_massProp.getValueDbl()),
  _height(_heightProp.getValueDbl()),
  _age(_ageProp.getValueDbl()),
  _notes(_notesProp.getValueStr()),
  _genericModelMakerProp(PropertyObj("", GenericModelMaker())),
  _genericModelMaker((GenericModelMaker&)_genericModelMakerProp.getValueObj()),
  _oSensorPlacerProp(PropertyObj("", OrientationSensorPlacer())),
  _oSensorPlacer((OrientationSensorPlacer&)_oSensorPlacerProp.getValueObj())
{
  setNull();
  setupProperties();
  copyData(aSubject);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ScaleTool to another.
 *
 * @param aSubject ScaleTool to be copied.
 */
void OrientationSensorsPlacerTool::copyData(const OrientationSensorsPlacerTool &aSubject)
{
  _mass = aSubject._mass;
  _height = aSubject._height;
  _age = aSubject._age;
  _notes = aSubject._notes;
  _genericModelMaker = aSubject._genericModelMaker;
  _oSensorPlacer = aSubject._oSensorPlacer;
}

//_____________________________________________________________________________
/**
 * Set the data members of this ScaleTool to their null values.
 */
void OrientationSensorsPlacerTool::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void OrientationSensorsPlacerTool::setupProperties()
{
  _massProp.setComment("Mass of the subject in kg.  For informational purposes only (not used by scaling)");
  _massProp.setName("mass");
  _massProp.setValue(-1.0);
  _propertySet.append(&_massProp);

  _heightProp.setComment("Height of the subject in mm.  For informational purposes only (not used by scaling).");
  _heightProp.setName("height");
  _heightProp.setValue(-1.0);
  _propertySet.append(&_heightProp);

  _ageProp.setComment("Age of the subject in years.  For informational purposes only (not used by scaling).");
  _ageProp.setName("age");
  _ageProp.setValue(-1.0);
  _propertySet.append(&_ageProp);

  _notesProp.setComment("Notes for the subject.");
  _notesProp.setName("notes");
  _propertySet.append(&_notesProp);

  // TODO: overload the genericModelMaker to enable defining oSensors set input xml and removing marker set
  _genericModelMakerProp.setComment("Specifies the name of the input model (.osim) and the marker set.");
  _genericModelMakerProp.setName("GenericModelMaker");
  _propertySet.append(&_genericModelMakerProp);

  _oSensorPlacerProp.setComment("Specifies parameters for orienting oSensor on the model.");
  _oSensorPlacerProp.setName("OrientationSensorPlacer");
  _propertySet.append(&_oSensorPlacerProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void OrientationSensorsPlacerTool::registerTypes()
{
  Object::registerType(GenericModelMaker());
  Object::registerType(OrientationSensorPlacer());
  Object::registerType(OrientationSensor());
  GenericModelMaker::registerTypes();
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
OrientationSensorsPlacerTool& OrientationSensorsPlacerTool::operator=(const OrientationSensorsPlacerTool &aSubject)
{
  // BASE CLASS
  Object::operator=(aSubject);

  copyData(aSubject);

  return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Create a generic model, using GenericModelMaker::processModel().
 *
 * @return Pointer to the Model that is created.
 */
Model* OrientationSensorsPlacerTool::createModel() {
  /* Make the generic model. */
  if (!_genericModelMakerProp.getValueIsDefault()){
    Model *model = _genericModelMaker.processModel(_pathToSubject);
    if (!model) {
      cout << "===ERROR===: Unable to load generic model." << endl;
      return 0;
    }
    else {
      std::string newModelName = model->getName()+ " ";
      if(getName() == "default")
        newModelName += "Placer";
      else
        newModelName += getName();
      model->setName(newModelName);
      return model;
    }
  }
  else {
    cout << "OrientationSensorsPlacerTool.createModel: WARNING- Unscaled model not specified (" << _genericModelMakerProp.getName();
    cout << " section missing from setup file)." << endl;
  }
  return 0;
}
