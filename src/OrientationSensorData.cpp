/* -------------------------------------------------------------------------- *
 *                          OpenSim:  OrientationSensorData.cpp                          *
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
        size_t dotIndex = sensorNameWithSuffix.toLower().find_last_of(".w");
        SimTK::String candidateOSensorName = sensorNameWithSuffix.substr(0, dotIndex - 1);
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

// TODO: find a smart way to average a set of quaternions
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
//void OrientationSensorData::averageFrames(double aThreshold, double aStartTime, double aEndTime) // TODO: check the implementation
//{
//    if (_numFrames < 2)
//        return;
//
//    int startIndex = 0, endIndex = 1;
//    double *minW = NULL, *minX = NULL, *minY = NULL, *minZ = NULL, *maxW = NULL, *maxX = NULL, *maxY = NULL, *maxZ = NULL;
//
//    findFrameRange(aStartTime, aEndTime, startIndex, endIndex);
//    OrientationSensorFrame *averagedFrame = new OrientationSensorFrame(*_frames[startIndex]);
//
//    /* If aThreshold is greater than zero, then calculate
//     * the movement of each sensor so you can check if it
//     * is greater than aThreshold.
//     */
//    if (aThreshold > 0.0)
//    {
//        minW = new double [_numOrientationSensors];
//        minX = new double [_numOrientationSensors];
//        minY = new double [_numOrientationSensors];
//        minZ = new double [_numOrientationSensors];
//        maxW = new double [_numOrientationSensors];
//        maxX = new double [_numOrientationSensors];
//        maxY = new double [_numOrientationSensors];
//        maxZ = new double [_numOrientationSensors];
//        for (int i = 0; i < _numOrientationSensors; i++)
//        {
//            minW[i] = minX[i] = minY[i] = minZ[i] = SimTK::Infinity;
//            maxW[i] = maxX[i] = maxY[i] = maxZ[i] = -SimTK::Infinity;
//        }
//    }
//
//    /* Initialize all the averaged sensor orientations to 1,0,0,0. Then
//     * loop through the frames to be averaged, adding each marker location
//     * to averagedFrame. Keep track of the min/max XYZ for each marker
//     * so you can compare it to aThreshold when you're done.
//     */
//    for (int i = 0; i < _numOrientationSensors; i++)
//    {
//        int numFrames = 0;
//        Vec3& avePt = averagedFrame->updOrientationSensor(i);
//        avePt = Vec3(0);
//
//        for (int j = startIndex; j <= endIndex; j++)
//        {
//            Vec3& pt = _frames[j]->updMarker(i);
//            if (!pt.isNaN())
//            {
//                Vec3& coords = pt; //.get();
//                avePt += coords;
//                numFrames++;
//                if (aThreshold > 0.0)
//                {
//                    if (coords[0] < minX[i])
//                        minX[i] = coords[0];
//                    if (coords[0] > maxX[i])
//                        maxX[i] = coords[0];
//                    if (coords[1] < minY[i])
//                        minY[i] = coords[1];
//                    if (coords[1] > maxY[i])
//                        maxY[i] = coords[1];
//                    if (coords[2] < minZ[i])
//                        minZ[i] = coords[2];
//                    if (coords[2] > maxZ[i])
//                        maxZ[i] = coords[2];
//                }
//            }
//        }
//
//        /* Now divide by the number of frames to get the average. */
//        if (numFrames > 0)
//            avePt /= (double)numFrames;
//        else
//            avePt = Vec3(SimTK::NaN) ;//(SimTK::NaN, SimTK::NaN, SimTK::NaN);
//    }
//
//    /* Store the indices from the file of the first frame and
//     * last frame that were averaged, so you can report them later.
//     */
//    int startUserIndex = _frames[startIndex]->getFrameNumber();
//    int endUserIndex = _frames[endIndex]->getFrameNumber();
//
//    /* Now delete all the existing frames and insert the averaged one. */
//    _frames.clearAndDestroy();
//    _frames.append(averagedFrame);
//    _numFrames = 1;
//    _firstFrameNumber = _frames[0]->getFrameNumber();
//
//    if (aThreshold > 0.0)
//    {
//        for (int i = 0; i < _numOrientationSensors; i++)
//        {
//            Vec3& pt = _frames[0]->updMarker(i);
//
//            if (pt.isNaN())
//            {
//                cout << "___WARNING___: marker " << _orientationSensorNames[i] << " is missing in frames " << startUserIndex
//                      << " to " << endUserIndex << ". Coordinates will be set to NAN." << endl;
//            }
//            else if (maxX[i] - minX[i] > aThreshold ||
//                      maxY[i] - minY[i] > aThreshold ||
//                      maxZ[i] - minZ[i] > aThreshold)
//            {
//                double maxDim = maxX[i] - minX[i];
//                maxDim = MAX(maxDim, (maxY[i] - minY[i]));
//                maxDim = MAX(maxDim, (maxZ[i] - minZ[i]));
//                cout << "___WARNING___: movement of marker " << _orientationSensorNames[i] << " in " << _fileName
//                      << " is " << maxDim << " (threshold = " << aThreshold << ")" << endl;
//            }
//        }
//    }
//
//    cout << "Averaged frames from time " << aStartTime << " to " << aEndTime << " in " << _fileName
//          << " (frames " << startUserIndex << " to " << endUserIndex << ")" << endl;
//
//    if (aThreshold > 0.0)
//    {
//        delete [] minX;
//        delete [] minY;
//        delete [] minZ;
//        delete [] maxX;
//        delete [] maxY;
//        delete [] maxZ;
//    }
//}

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
