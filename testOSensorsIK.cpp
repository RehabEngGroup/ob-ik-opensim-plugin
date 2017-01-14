#include <OpenSim/OpenSim.h>
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"
#include "OpenSim/Tools/IKExtendedTaskSet.h"
#include "OpenSim/Simulation/Model/OrientationSensor.h"
#include "OpenSim/Simulation/Model/OrientationSensorSet.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/ComponentSet.h"
#include <OpenSim/Simulation/Model/ModelVisualizer.h>

int main(int argc, char* argv[]) {

 // OpenSim::Object::registerType(OpenSim::OrientationSensor());
 // OpenSim::Object::registerType(OpenSim::IKExtendedTaskSet());
 // OpenSim::Object::registerType(OpenSim::IKOrientationSensorTask());
 // OpenSim::Object::registerType(OpenSim::InverseKinematicsExtendedTool());

  if (argc < 3) {
    std::cout << "Please provide a model and an Extended IK setup file" << std::endl;
    return 0;
  }

  std::string modelFileName = argv[1];
  OpenSim::Model model(modelFileName);
  std::cout << "Loaded model " << model.getName() << std::endl;
 /* OpenSim::ComponentSet &modelComponentSet = model.updMiscModelComponentSet();
  OpenSim::OrientationSensorSet modelOSensorSet;
  std::cout << modelComponentSet.getSize() << std::endl;
  for (int i = 0; i < modelComponentSet.getSize(); ++i) {
    OpenSim::OrientationSensor* oSens = dynamic_cast<OpenSim::OrientationSensor*>(&modelComponentSet.get(i));
    if (oSens) {
      oSens->connectOSensorToModel(model);
      std::string nameSens = oSens->getName();
      std::string nameBody = oSens->getBody().getName();
      std::cout << "Name: " << nameSens << " Body" << nameBody << std::endl;
    }
    else
      std::cout << "Not an osensor" << std::endl;
  }*/

  OpenSim::InverseKinematicsExtendedTool ik(argv[2]);

  ik.setModel(model);
  model.setUseVisualizer(true);
  // start timing
  std::clock_t startTime = std::clock();

  // RUN
  ik.run();

  std::cout << "IK compute time = " << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n";
  return 0;
}
