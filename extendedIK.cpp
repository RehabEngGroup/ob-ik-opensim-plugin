/* -------------------------------------------------------------------------- *
 *          Orientation Based Inverse Kinematics : extendedIK.cpp             *
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
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"

using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

int main(int argc, char **argv) {

  // Set output formats
  OpenSim::IO::SetDigitsPad(4);

  // Register types
  OpenSim::InverseKinematicsExtendedTool::registerTypes();

  string setupFilename;
  string option = "";
  bool useVisualizer = false;

  // No option provided, print usage options
  if (argc < 2) {
    PrintUsage(argv[0], cout);
    exit(EXIT_FAILURE);
  }

  // Parse command line arguments
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
        setupFilename = argv[i + 1];
        ++i;
        continue;
      }

      // Set if extendedIK uses Simbody visualizer
      else if ((option == "-V") || (option == "-UseVisualizer")) {
        useVisualizer = true;
        continue;
      }

      // Print a default setup file
      else if ((option == "-PrintSetup") || (option == "-PS")) {
        OpenSim::InverseKinematicsExtendedTool *ikTool = new OpenSim::InverseKinematicsExtendedTool();
        ikTool->setName("default");

        // Add in useful objects that may need to be instantiated
        OpenSim::Object::setSerializeAllDefaults(true);
        ikTool->print("default_Setup_Extended_Inverse_Kinematics.xml");
        OpenSim::Object::setSerializeAllDefaults(false);
        cout << "Created file default_Setup_Extended_Inverse_Kinematics.xml with default setup" << endl;
        delete ikTool;
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
      // Unrecognized
      else {
        cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
        PrintUsage(argv[0], cout);
        exit(EXIT_FAILURE);
      }
    }
  }

  // Create and run tool
  try {
    OpenSim::InverseKinematicsExtendedTool ikTool(setupFilename);

    ikTool.setUseVisualizer(useVisualizer);
    ikTool.run();

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
  aOStream << "Option                Argument          Action / Notes\n";
  aOStream << "------                --------          --------------\n";
  aOStream << "-Help, -H                               Print the command-line options for " << progName << ".\n";
  aOStream << "-PrintSetup, -PS                        Generates a template Setup file to customize extended IK. \n";
  aOStream << "-Setup, -S            SetupFileName     Specify an xml setup file for extended IK.\n";
  aOStream << "-UseVisualizer, -V                      Use Simbody visualizer. To be used together with -S option.\n";
  aOStream << "-PropertyInfo, -PI                      Print help information for properties in setup files.\n";
}
