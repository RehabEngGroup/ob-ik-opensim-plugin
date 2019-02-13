Orientation Based Inverse Kinematics (OB-IK) Plug-in for OpenSim
================================================================


Overview
--------
Wearable and relatively cheap, Inertial (and Magnetic) Measurement Units (IMUs) are a promising solution to human motion estimation. Indeed, this technology could enable continuous kinematics analysis in almost every environment (i.e. clinical, outdoor, daily-life, industry).

This project extends the OpenSim Inverse Kinematics Tool, enabling the estimation of joint kinematics from orientations provided by IMUs or from a combination of IMUs orientations and markers.


The developed OB-IK plug-in for OpenSim contains:

+ *oSensor model*: a model of the orientation sensor
+ *extendedIKTool*: an extended version of the OpenSim Inverse Kinematics tool, which enables to solve the IK problem using as data source either oSensor orientations or marker data (or both of them as the same time)
+ *oSensorsPlacerTool*: a tool to adjust the orientation of the virtual oSensors placed on a model, trying to match the experimental orientations collected during a static trial. Requirement for this tool is the knowledge of the pose of the subject during the static trial.


Moreover, two executables are provided to run the tools from command line. The configuration of the execution can be easily managed using .xml setup files, as for the standard OpenSim tools.


Test data
---------
The folder test_data includes some test data taken from the Framework 1 of the work you are kindly asked to cite if using this 
plug-in.
Test data are provided as examples, and we ask you to do not use the provided model without asking for permission and without citing 
the related paper, since it can be considered as covered by Creative Commons License.

Requirements
-----------
+ OpenSim v. 3.3
+ CMake ( >= 2.8.12)
+ C++ compiler supporting C++11 (gcc >= 4.2 , Visual Studio > 2013, CLang >= 3.3 )


GUI Support
-----------
Since OpenSim Model class has not been extended, oSensors are interpreted by the model as MiscComponents, and it is not possible to visualize virtual orientation sensors within the GUI. This issue depends on the missing capability of OpenSim GUI to display MiscComponents.
However, oSensors are correctly displayed in Simbody's Visualizer.


Licensing
--------------
The Orientation Based Inverse Kinematics Plug-in for OpenSim is distributed under the terms of the Apache 2.0 license (see LICENSE.txt and NOTICE.txt), making it suitable for any use, commercial, government, academic, or personal.

## Citing this work

Please cite the following publication if you are using the code contained in this repository for your own research and/or experiments


~~~
Validation of a model-based inverse kinematics approach based on wearable inertial sensors
Luca Tagliapietra, Luca Modenese, Elena Ceseracciu, Claudia Mazzá, Monica Reggiani
Computer Methods in Biomechanics and Biomedical Engineering
DOI:  10.1080/10255842.2018.1522532
~~~

The bibtex code for including this citation is provided:
~~~
@article{tagliapietra2018Validation,
  title={Validation of a model-based inverse kinematics approach based on wearable inertial sensors},
  author={Tagliapietra, Luca and Modenese, Luca and Ceseracciu, Elena and Mazz'a, Claudia and Reggiani, Monica},
  year={2018},
  DOI={10.1080/10255842.2018.1522532},
  publisher={Taylor \& Frencis}, 
  journal={Computer Methods in Biomechanics and Biomedical Engineering}, 
}
~~~


Notes
--------
Tested on:

+ Mac OSX 10.12 (Sierra) - CMake v.3.7.2 - XCode 8.3.1 (Apple CLang 8.1.0)
+ Mac OSX 10.12 (Sierra) - CMake v.3.7.2 - gcc 4.2.1
+ Windows 10 64 bits - CMake v.3.7.2 - Visual Studio 14 (2015)
+ Ubuntu Linux 15.04 64 bits - CMake v.3.0.2 - gcc 4.9.2
