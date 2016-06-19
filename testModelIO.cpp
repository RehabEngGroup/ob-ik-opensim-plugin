#include <OpenSim/OpenSim.h>
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"
#include "OpenSim/Tools/IKExtendedTaskSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"

int main(int argc, char* argv[]) {

  OpenSim::Object::registerType(OpenSim::OrientationSensor());
  OpenSim::Object::registerType(OpenSim::IKExtendedTaskSet());
  OpenSim::Object::registerType(OpenSim::IKOrientationSensorTask());
  OpenSim::Object::registerType(OpenSim::InverseKinematicsExtendedTool());

  if (argc < 2) {
    std::cout << "Please provide a model" << std::endl;
    return 0;
  }

  std::string modelFileName = argv[1];
  OpenSim::Model model(modelFileName);
  std::cout << "Loaded model " << model.getName() << std::endl;
  OpenSim::ComponentSet &modelComponentSet = model.updMiscModelComponentSet();
  OpenSim::OrientationSensorSet modelOSensorSet;
  std::cout << modelComponentSet.getSize() << std::endl;
  for (int i = 0; i < modelComponentSet.getSize(); ++i) {
    OpenSim::OrientationSensor* oSens = dynamic_cast<OpenSim::OrientationSensor*>(&modelComponentSet.get(i));
    if (oSens) {
      std::string nameSens = oSens->getName();
      oSens->connectOSensorToModel(model);
      std::string nameBody = oSens->getBody().getName();
      std::cout << "Name: " << nameSens << " Body" << nameBody << std::endl;
    }
    else
      std::cout << "Not an osensor" << std::endl;
  }

  return 0;
}
