/* -------------------------------------------------------------------------- *
 *         Orientation Based Inverse Kinematics : testModelIO.cpp             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2016-2017 L. Tagliapietra, E. Ceseracciu, M. Reggiani        *
 *                                                                            *
 * Author(s): L. Tagliapietra (Mar 2016)                                      *
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

#include <OpenSim/OpenSim.h>
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"

int main(int argc, char* argv[]) {

  OpenSim::OrientationSensor::registerTypes();

  if (argc < 2) {
    std::cout << "Please provide a model" << std::endl;
    return 0;
  }

  std::string modelFileName = argv[1];
  OpenSim::Model model(modelFileName);
  std::cout << "Loaded model " << model.getName() << std::endl;
  OpenSim::ComponentSet &modelComponentSet = model.updMiscModelComponentSet();
  OpenSim::OrientationSensorSet modelOSensorSet;
  std::cout << modelComponentSet.getName() << std::endl;
  for (int i = 0; i < modelComponentSet.getSize(); ++i) {
    OpenSim::OrientationSensor* oSens = dynamic_cast<OpenSim::OrientationSensor*>(&modelComponentSet.get(i));
    if (oSens) {
      modelOSensorSet.cloneAndAppend(*oSens);
      std::string nameSens = oSens->getName();
      oSens->connectToModel(model);
      std::string nameBody = oSens->getBody().getName();
      std::cout << "Name: " << nameSens << " Body" << nameBody << std::endl;
    }
    else
      std::cout << "Not an osensor" << std::endl;
  }
    modelOSensorSet.setName("OrientationSensorSet_from_model_"+model.getName());
    modelOSensorSet.print("oSensorSet.xml");
  return 0;
}
