/* -------------------------------------------------------------------------- *
 *       Orientation Based Inverse Kinematics: OrientationSensorData.cpp      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2016-2017 L. Tagliapietra, E. Ceseracciu, M. Reggiani        *
 *                                                                            *
 * Author(s): L. Tagliapietra, E. Ceseracciu (Mar 2016)                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <fstream>
#include <math.h>
#include <float.h>
#include "OpenSim/Common/OrientationSensorData.h"
#include "OpenSim/Common/SimmIO.h"
#include "OpenSim/Common/SimmMacros.h"
#include "SimTKcommon.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationSensorData::OrientationSensorData() :
    _numFrames(0),
    _numOrientationSensors(0),
    _orientationSensorNames("")
{
}

//_____________________________________________________________________________
/**
 * Constructor from a MOT/STO file.
 */
OrientationSensorData::OrientationSensorData(const string& aFileName) :
    _numFrames(0),
    _numOrientationSensors(0),
    _orientationSensorNames("")
{

#if 0
   if (!aFileName)
        return;

   if (!lookForFile(aFileName, gWorkingDir.c_str(), actualFilename))
   {
      return smFileError;
   }
#endif

   /* Check if the suffix is MOT or STO */
    string suffix;
   int dot = (int)aFileName.find_last_of(".");
   suffix.assign(aFileName, dot+1, 3);
   SimTK::String sExtension(suffix);
   if (sExtension.toLower() == "mot" || sExtension.toLower() == "sto")
       readStoFile(aFileName);
   else
       throw Exception("OrientationSensorData: ERROR- Orientation Sensor file type is unsupported",__FILE__,__LINE__);

   _fileName = aFileName;

    cout << "Loaded Orientation Sensor file " << _fileName << " (" << _numOrientationSensors << " Orientation Sensors, " << _numFrames << " frames)" << endl;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationSensorData::~OrientationSensorData()
{
}

//=============================================================================
// I/O
//=============================================================================
//_____________________________________________________________________________

/**
 * Read sto file.
 *
 * @param aFilename name of sto file.
 */
void OrientationSensorData::readStoFile(const string& aFileName)
{

    if (aFileName.empty())
        throw Exception("OrientationSensorData.readStoFile: ERROR- sensor file name is empty",__FILE__,__LINE__);

    Storage store(aFileName);

    // populate map between sensor names and column numbers
    std::map<int, std::string>  sensorIndices;
    buildOSensorMap(store, sensorIndices);

    if (sensorIndices.size() == 0){
        throw Exception("OrientationSensorData.readStoFile: ERROR- No orientation sensors were identified. Sensors should appear on consecutive columns as OSensor1.w OSensor1.x OSensor1.y OSensor1.z... etc.",__FILE__,__LINE__);
    }
    std::map<int, std::string>::iterator iter;

    for (iter = sensorIndices.begin(); iter != sensorIndices.end(); iter++) {
        SimTK::String sensorNameWithSuffix = iter->second;
        SimTK::String originalName = sensorNameWithSuffix;
        size_t dotIndex = sensorNameWithSuffix.toLower().find_last_of(".w");
        SimTK::String candidateOSensorName = originalName.substr(0, dotIndex - 1);
        _orientationSensorNames.append(candidateOSensorName);
    }
    // use map to populate data for OrientationSensorData header
    // TODO: needs to be fixed, .sto does not have the header but dataRate is quite usefull
    int sz = store.getSize();

    _numOrientationSensors = (int) sensorIndices.size();
    _numFrames = store.getSize();
    _firstFrameNumber = 1;
    _dataRate = _numFrames / (store.getLastTime() - store.getFirstTime());
    _originalDataRate = _dataRate;
    _originalStartFrame = 1;
    _originalNumFrames = _numFrames;
    _fileName = aFileName;
    _units = Units(Units::UnknownUnits);

    double time;
    for (int i=0; i < sz; i++){
        StateVector* nextRow = store.getStateVector(i);
        time = nextRow->getTime();
        int frameNum = i+1;
        OrientationSensorFrame *frame = new OrientationSensorFrame(_numOrientationSensors, frameNum, time, _units);
        const Array<double>& rowData = nextRow->getData();
        // Cycle through map and add sensor coordinates to the frame. Same order as header.
        for (iter = sensorIndices.begin(); iter != sensorIndices.end(); iter++) {
            int startIndex = iter->first; // startIndex includes time but data doesn't!
            frame->addOrientationSensor(SimTK::Quaternion(rowData[startIndex - 1], rowData[startIndex], rowData[startIndex + 1], rowData[startIndex + 2]));
        }
        _frames.append(frame);
   }
}
/**
 * Helper function to check column labels of passed in Storage for possibly being a OrientationSensorName, and if true
 * add the start index and corresponding name to the passed in std::map
 */
void OrientationSensorData::buildOSensorMap(const Storage& storageToReadFrom, std::map<int, std::string>& sensorNames)
{
    const Array<std::string> & labels = storageToReadFrom.getColumnLabels();
    for (int i=1; i < labels.getSize()-3; i++){
        // if label ends in .W, check that three labels that follow are .X, .Y, .Z (case insensitive) with common prefix
        // if so, add to map
        SimTK::String nextLabel(labels.get(i));
        size_t dotIndex = nextLabel.toLower().find_last_of(".w");
        if (dotIndex > 1){  // possible sensor
            SimTK::String candidateSensorName = nextLabel.substr(0, dotIndex-1);
            // this may be replaced with getColumnIndicesForIdentifier(candidatesensorName) but this will be more permissive
            // as it allows for non-consecutive columns, could be non-triplet,...etc.
            SimTK::String nextLabel2 = labels.get(i+1);
            SimTK::String nextLabel3 = labels.get(i+2);
            SimTK::String nextLabel4 = labels.get(i+3);
            if ((nextLabel2.toLower() == candidateSensorName + ".x") && (nextLabel3.toLower() == candidateSensorName + ".y") && (nextLabel4.toLower() == candidateSensorName + ".z")){
                sensorNames[i] = labels.get(i); // this includes trailing .w
                i+= 3;
            }
        }
    }
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Find the range of frames that is between start time and end time
 * (inclusive).
 *
 * @param aStartTime start time.
 * @param aEndTime end time.
 * @param rStartFrame index of start frame is returned here.
 * @param rEndFrame index of end frame is returned here.
 */
void OrientationSensorData::findFrameRange(double aStartTime, double aEndTime, int& rStartFrame, int& rEndFrame) const
{
    int i;

    rStartFrame = 0;
    rEndFrame = _numFrames - 1;

    if (aStartTime > aEndTime)
    {
        throw Exception("OrientationSensorData: findFrameRange start time is past end time.");
    }

    for (i = _numFrames - 1; i >= 0 ; i--)
    {
        if (_frames[i]->getFrameTime() <= aStartTime)
        {
            rStartFrame = i;
            break;
        }
    }

    for (i = rStartFrame; i < _numFrames; i++)
    {
        if (_frames[i]->getFrameTime() >= aEndTime - SimTK::Zero)
        {
            rEndFrame = i;
            break;
        }
    }
}
//_____________________________________________________________________________
/**
 * Utilities to support the GUI
 *
 * getStartFrameTime: Exposes the time for first frame
 */

double OrientationSensorData::getStartFrameTime() const
{
    if (_numFrames<=0)
        return SimTK::NaN;

    return(_frames[0]->getFrameTime());

}
/**
 * Utilities to support the GUI
 *
 * getLastFrameTime: Expose the time for the last frame
 */
double OrientationSensorData::getLastFrameTime() const
{
    if (_numFrames<=0)
        return SimTK::NaN;

    return(_frames[_numFrames-1]->getFrameTime());
}

//_____________________________________________________________________________
/**
 * Average all the frames between aStartTime and
 * aEndTime (inclusive) and store the result in the first
 * frame. All other frames are deleted. The time and frame
 * number of this one remaining frame are copied from the
 * startIndex frame. The aThreshold parameter is for printing
 * a warning if any sensor moves more than that amount in
 * the averaged frames. aThreshold is specified by the user,
 * and is assumed to be in the units of the sensor data.
 *
 * @param aThreshold amount of sensor movement that is allowed for averaging.
 * @param aStartTime start time of frame range to average.
 * @param aEndTime end time of frame range to average.
 */
void OrientationSensorData::averageFrames(double aThreshold, double aStartTime, double aEndTime) {
  if (_numFrames < 2)
    return;

  int startIndex = 0, endIndex = 1;
  findFrameRange(aStartTime, aEndTime, startIndex, endIndex);

  OrientationSensorFrame *averagedFrame = new OrientationSensorFrame(*_frames[startIndex]);

  for (int i = 0; i < _numOrientationSensors; i++) {
    SimTK::Mat44 A;
    A.setToZero();
    int nEffectiveFrames = 0;
    SimTK::Quaternion& avg = averagedFrame->updOrientationSensor(i);
    avg = SimTK::Quaternion();
    for (int j = startIndex; j <= endIndex; j++) {
      SimTK::Quaternion q = _frames.get(j)->updOrientationSensor(i);
      if (!(q.isNaN() || q.isInf())) {
        A = q * ~q + A;
        nEffectiveFrames++;
      }
    }
    A = (1.0 / nEffectiveFrames) * A;
    SimTK::Matrix a(A);
    SimTK::Eigen eigen(a);
    SimTK::Vector_<std::complex<double> > values; // should get sized automatically to 4 by getAllEigenValuesAndVectors()
    SimTK::Matrix_<std::complex<double> > vectors;
    eigen.getAllEigenValuesAndVectors(values, vectors);
    double max = 0.0;
    int maxIndex = 0;
    for (int w = 0; w < values.size(); ++w) {
      if (values[w].real() > max) {
        max = values[w].real();
        maxIndex = w;
      }
    }
    for(int k=0; k< 4; ++k)
      avg[k] = vectors.col(maxIndex).getElt(k, maxIndex).real();
  }

  int startUserIndex = _frames[startIndex]->getFrameNumber();
  int endUserIndex = _frames[endIndex]->getFrameNumber();
  std::cout << "Averaged frames from time " << aStartTime << " to " << aEndTime << " in " << _fileName
       << " (frames " << startUserIndex << " to " << endUserIndex << ")" << std::endl;
  _frames.clearAndDestroy();
  _frames.append(averagedFrame);
  _numFrames = 1;
  _firstFrameNumber = _frames[0]->getFrameNumber();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get a frame of sensor data.
 *
 * @param aIndex index of the row to get.
 * @return Pointer to the frame of data.
 */
const OrientationSensorFrame& OrientationSensorData::getFrame(int aIndex) const
{
    if (aIndex < 0 || aIndex >= _numFrames)
        throw Exception("OrientationSensorData::getFrame() invalid frame index.");
    return *_frames[aIndex];
}

//_____________________________________________________________________________
/**
 * Get the index of a sensor, given its name.
 *
 * @param aName name of sensor.
 * @return Index of the named sensor.
 */
int OrientationSensorData::getOrientationSensorIndex(const string& aName) const
{
    for (int i = 0; i < _orientationSensorNames.getSize(); i++)
    {
        if (_orientationSensorNames[i] == aName)
            return i;
    }
    return -1;
}
