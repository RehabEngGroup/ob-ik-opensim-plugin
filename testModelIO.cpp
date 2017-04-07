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
