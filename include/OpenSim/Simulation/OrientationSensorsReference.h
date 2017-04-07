/* -------------------------------------------------------------------------- *
 *    Orientation Based Inverse Kinematics : OrientationSensorsReference.h    *
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

#ifndef __OrientationSensorsReference_h__
#define __OrientationSensorsReference_h__

#include "OpenSim/Simulation/Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/OrientationSensorData.h>

namespace OpenSim {
    template <> struct Object_GetClassName<SimTK::Quaternion>
    {
        static const std::string name() { return "Quaternion"; }
    };
class Units;

class OSIMEXTENDEDIK_API OrientationSensorWeight : public Object {
	OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorWeight, Object);

private:
    PropertyDbl _weightProp;
    double &_weight;

public:
	  OrientationSensorWeight() : Object(), _weight(_weightProp.getValueDbl()) {}

    OrientationSensorWeight(std::string name, double weight)
    :   Object(), _weight(_weightProp.getValueDbl())
    {   setName(name); _weight = weight; }

    //Copy
    OrientationSensorWeight(const OrientationSensorWeight& source)
    :   Object(source), _weight(_weightProp.getValueDbl())
    {   _weight = source._weight; }

    #ifndef SWIG
    OrientationSensorWeight& operator=(const OrientationSensorWeight& source) {
        if (&source != this) {
            Super::operator=(source);
            _weight = source._weight;
        }
        return *this;
    }
    #endif

    void setWeight(double weight) {_weight = weight; }
    double getWeight() const {return _weight; };

}; // end of OrientationSensorWeight class


//=============================================================================
//=============================================================================
/**
 * Reference values to be achieved for specified OSensor that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one OSensor's reference relative to
 * another.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class IKTaskSet;

class OSIMEXTENDEDIK_API OrientationSensorsReference : public Reference_<SimTK::Quaternion> {
OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensorsReference, Reference_<SimTK::Quaternion>);


//=============================================================================
// MEMBER VARIABLES
//=============================================================================

protected:

private:
    // Implementation details

    // Use a specialized data structure for holding the orientation sensors data
    SimTK::ReferencePtr<OrientationSensorData> _orientationSensorData;
    // marker names inside the orientation sensors data
    SimTK::Array_<std::string> _orientationSensorNames;
    // corresponding list of weights guaranteed to be in the same order as names above
    SimTK::Array_<double> _weights;

//=============================================================================
// METHODS
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(osensors_file, std::string,
        "MOT/STO file containing the time history of observations of sensor orientations.");
    OpenSim_DECLARE_LIST_PROPERTY(osensor_weights, OrientationSensorWeight,
        "Set of sensor weights identified by sensor name with weight being a positive scalar.");
    OpenSim_DECLARE_PROPERTY(default_weight, double,
        "");
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    OrientationSensorsReference();

    // Convenience load markers from a file
		OrientationSensorsReference(const std::string filename);

		OrientationSensorsReference(OrientationSensorData& aOrientationSensorData, const Set<OrientationSensorWeight>* aOrientationSensorWeightSet = NULL);

	//	OrientationSensorsReference& operator=(const OrientationSensorsReference &aRef) { Reference_<SimTK::Vec3>::operator=(aRef); copyData(aRef); return(*this); };

		virtual ~OrientationSensorsReference() {}

    /** load the orientation sensors data for this OrientationSensorsReference from OrientationSensorsFile  */
    void loadOrientationSensorsFile(const std::string orientationSensorsFile);


    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
		int getNumRefs() const override { return _orientationSensorData->getNumOrientationSensors(); }
    /** get the time range for which the OrientationSensorsReference values are valid,
        based on the loaded marker data.*/
    SimTK::Vec2 getValidTimeRange() const override;
    /** get the names of the orientation sensors serving as references */
    const SimTK::Array_<std::string>& getNames() const override;
    /** get the value of the OrientationSensorsReference */
    void getValues(const SimTK::State &s, SimTK::Array_<SimTK::Quaternion> &values) const override;
    /** get the speed value of the OrientationSensorsReference */
    virtual void getSpeedValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &speedValues) const;
    /** get the acceleration value of the OrientationSensorsReference */
    virtual void getAccelerationValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &accValues) const;
    /** get the weighting (importance) of meeting this OrientationSensorsReference in the same order as names*/
    void getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const override;

    //--------------------------------------------------------------------------
    // Convenience Access
    //--------------------------------------------------------------------------
    double getSamplingFrequency() {return _orientationSensorData->getDataRate(); }
    Set<OrientationSensorWeight> getOrientationSensorWeightSet();
    void setOrientationSensorWeightSet(const Set<OrientationSensorWeight> &orientationSensorWeights);
    void setDefaultWeight(double weight) {set_default_weight(weight); }

private:
    // utility to define object properties including their tags, comments and
    // default values.
    void constructProperties();

    void populateFromOrientationSensorData(OrientationSensorData& aOrientationSensorData);

//=============================================================================
};  // END of class OrientationSensorsReference
//=============================================================================
} // namespace

#endif // __OrientationSensorsReference_h__
