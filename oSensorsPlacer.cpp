/* -------------------------------------------------------------------------- *
 *                          testOSensorPlacer.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
 * testOSensorPlacer.cpp
 * Test program to refine orientation of oSensors in the model
 *
 * @author: Luca Tagliapietra <tagliapietra@gest.unipd.it>
 * @notes:  2016, Dec
 */

 // INCLUDES

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "OpenSim/Tools/OrientationSensorsPlacerTool.h"

using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

int main(int argc, char **argv) {

  // Set output format
  OpenSim::IO::SetDigitsPad(4);

  // Register types
  OpenSim::OrientationSensorsPlacerTool::registerTypes();

  string inName;
  string option = "";

  // No option provided, print usage options
  if (argc < 2) {
    PrintUsage(argv[0], cout);
    exit(EXIT_FAILURE);
  }

  // Parse command line
  else {
    for (int i = 1; i <= (argc - 1); i++) {
      option = argv[i];

      // Print usage options
      if ((option == "-help") || (option == "-h") || (option == "-Help") || (option == "-H") ||
        (option == "-usage") || (option == "-u") || (option == "-Usage") || (option == "-U")) {
        PrintUsage(argv[0], cout);
        exit(EXIT_SUCCESS);
      }

      // Identify the setup file
      else if ((option == "-S") || (option == "-Setup")) {
        if (argv[i + 1] == 0) {
          PrintUsage(argv[0], cout);
          exit(EXIT_FAILURE);
        }
        inName = argv[i + 1];
        break;
      }

      // Print a default setup file
      else if ((option == "-PrintSetup") || (option == "-PS")) {
        OpenSim::OrientationSensorsPlacerTool *subject = new OpenSim::OrientationSensorsPlacerTool();
        subject->setName("default");

        // Add in useful objects that may need to be instantiated
        OpenSim::Object::setSerializeAllDefaults(true);
        subject->print("default_Setup_oSensorPlacer.xml");
        OpenSim::Object::setSerializeAllDefaults(false);
        cout << "Created file default_Setup_oSensorPlacer.xml with default setup" << endl;
        exit(EXIT_SUCCESS);
      }

      // Print property info
      else if ((option == "-PropertyInfo") || (option == "-PI")) {
        if ((i + 1) >= argc)
          OpenSim::Object::PrintPropertyInfo(cout, "");
        else {
          char *compoundName = argv[i + 1];
          if (compoundName[0] == '-')
            OpenSim::Object::PrintPropertyInfo(cout, "");
          else
            OpenSim::Object::PrintPropertyInfo(cout, compoundName);
        }
        exit(EXIT_SUCCESS);
      }

      // Unrecognized argument
      else {
        cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
        PrintUsage(argv[0], cout);
        exit(EXIT_FAILURE);
      }
    }
  }

  // Create and run tool
  try {
    // Read oSensorPlacer parameters and construct model
    OpenSim::OrientationSensorsPlacerTool* scaleTool = new OpenSim::OrientationSensorsPlacerTool(inName);
    OpenSim::Model* model = scaleTool->createModel();

    if (!model) throw OpenSim::Exception("oSensorPlacer: ERROR- No model specified.", __FILE__, __LINE__);

    SimTK::State& state = model->initSystem();

    if (!scaleTool->isDefaultOrientationSensorPlacer()) {
      OpenSim::OrientationSensorPlacer& placer = scaleTool->getOrientationSensorPlacer();

      if (!placer.processModel(state, model, scaleTool->getPathToSubject()))
        exit(EXIT_FAILURE);
    }
    else
      cout << "OSensor placement parameters disabled (apply is false) or not set. No oSensors have been moved." << endl;

    delete model;
    delete scaleTool;
    exit(EXIT_SUCCESS);
  }
  catch (const OpenSim::Exception& x) {
    x.print(cout);
    exit(EXIT_FAILURE);
  }
}

//_____________________________________________________________________________
/**
* Print the usage for this application
*/
void PrintUsage(const char *aProgName, ostream &aOStream) {
  string progName = OpenSim::IO::GetFileNameFromURI(aProgName);
  aOStream << "\n\n" << progName << "\n\n";
  aOStream << "Option            Argument          Action / Notes\n";
  aOStream << "------            --------          --------------\n";
  aOStream << "-Help, -H                           Print the command-line options for " << progName << ".\n";
  aOStream << "-PrintSetup, -PS                    Generates a template Setup file to customize oSensor placing.\n";
  aOStream << "-Setup, -S        SetupFileName     Specify an xml setup file for oSensors placing.\n";
  aOStream << "-PropertyInfo, -PI                  Print help information for properties in setup files.\n";
}
