/* -------------------------------------------------------------------------- *
 *   Orientation Based Inverse Kinematics: OrientationSensorsPlacerTool.cpp   *
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

#include "OpenSim/Tools/OrientationSensorsPlacerTool.h"
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"
#include <OpenSim/Common/SimmIO.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "SimTKsimbody.h"

using namespace std;
using namespace OpenSim;

//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationSensorsPlacerTool::OrientationSensorsPlacerTool() :
  _mass(_massProp.getValueDbl()),
  _height(_heightProp.getValueDbl()),
  _age(_ageProp.getValueDbl()),
  _notes(_notesProp.getValueStr()),
  _modelFileName(_modelFileNameProp.getValueStr()),
  //_oSensorSetFileName(_oSensorSetFileNameProp.getValueStr()),
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
  _modelFileName(_modelFileNameProp.getValueStr()),
  //_oSensorSetFileName(_oSensorSetFileNameProp.getValueStr()),
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
  _modelFileName(_modelFileNameProp.getValueStr()),
  //_oSensorSetFileName(_oSensorSetFileNameProp.getValueStr()),
  _oSensorPlacerProp(PropertyObj("", OrientationSensorPlacer())),
  _oSensorPlacer((OrientationSensorPlacer&)_oSensorPlacerProp.getValueObj())
{
  setNull();
  setupProperties();
  copyData(aSubject);
}

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
  _modelFileName = aSubject._modelFileName;
  //_oSensorSetFileName = aSubject._oSensorSetFileName;
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

  _modelFileNameProp.setComment("Model file (.osim) for the subject.");
  _modelFileNameProp.setName("model_file");
  _propertySet.append(&_modelFileNameProp);

  // TODO: Check if make sense, and if so add this property
  // TODO: Requires to fix the constructor from xml file

  //_oSensorSetFileNameProp.setComment("Set of oSensors (.xml) to be used.");
  //_oSensorSetFileNameProp.setName("oSensor_set_file");
  //_propertySet.append(&_oSensorSetFileNameProp);

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
  Object::registerType(OrientationSensorPlacer());
  Object::registerType(OrientationSensor());
}

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

//_____________________________________________________________________________
/**
 * Load a model
 *
 * @return Pointer to the Model that is created.
 */
Model* OrientationSensorsPlacerTool::createModel() {

  Model* model = NULL;

  cout << endl << "Step 1: Loading model" << endl;

  try
  {
		_modelFileName = _pathToSubject + _modelFileName;

		model = new Model(_modelFileName);
		model->initSystem();
  }
  catch (const Exception& x) {
		x.print(cout);
  }

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
