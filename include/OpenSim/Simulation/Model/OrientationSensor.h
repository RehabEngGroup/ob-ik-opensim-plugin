/* -------------------------------------------------------------------------- *
 *        Orientation Based Inverse Kinematics : OrientationSensor.h          *
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

#ifndef _OrientationSensor_h
#define _OrientationSensor_h

#include <iostream>
#include <math.h>
#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include "SimTKcommon.h"

namespace OpenSim {

    class Body;
    class Model;
    class VisibleObject;

    class OSIMEXTENDEDIK_API OrientationSensor : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensor, ModelComponent);
        OpenSim_DECLARE_PROPERTY(fixed, bool,
            "Flag (true or false) specifying whether or not an orientation sensor "
        "should be kept fixed in the orientation sensor placement step.  i.e. If false, the orientation sensor is allowed to move.");
        OpenSim_DECLARE_PROPERTY(body, std::string,
            "Body segment in the model on which the orientation sensor resides.");
        OpenSim_DECLARE_PROPERTY(position_offset, SimTK::Vec3,
            "Location of the orientation sensor on the body segment.");
        OpenSim_DECLARE_PROPERTY(rotation_offset, SimTK::Vec3,
            "Rotation of the orientation sensor frame with respect to the body one.");

        class Body;

    private:

    protected:
        const Model* _model;

        // Body that the OSensor is attached to
        OpenSim::Body* _body;

        // Support for Display
        VisibleObject _displayer;

        /** A temporary kluge until the default mechanism is working */
        static Geometry *_defaultGeometry;
        bool _virtual;

    public:
        OrientationSensor();
        OrientationSensor(const OrientationSensor &aOSensor);
        static void registerTypes();
        virtual ~OrientationSensor();

        static void deleteOSensor(OrientationSensor* aOSensor) { if (aOSensor) delete aOSensor; }

//#ifndef SWIG
        OrientationSensor& operator=(const OrientationSensor &aOSensor);
//#endif
        void copyData(const OrientationSensor &aOSensor);

        virtual void updateFromOSensor(const OrientationSensor &aOSensor);
        virtual OpenSim::Body& getBody() const { return *_body; }
        virtual void changeBody(OpenSim::Body& aBody);
        virtual void changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody);
        virtual void scale(const SimTK::Vec3& aScaleFactors);
        virtual void connectToModel(Model& aModel) OVERRIDE_11;
        void setPositionOffset(const SimTK::Vec3& offset);
        void setRotationOffset(const SimTK::Vec3& offset);
        SimTK::Vec3 getPositionOffset();
        SimTK::Vec3 getRotationOffset();
        void setBodyName(const std::string aBodyName);
        virtual void updateGeometry();

        virtual const VisibleObject* getDisplayer() const { return &_displayer; }
        virtual VisibleObject*	updDisplayer() { return &_displayer; };
        virtual void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const OVERRIDE_11;

        virtual void removeSelfFromDisplay();
        const bool isVirtual()
        {
            return _virtual;
        }
        void setVirtual(bool aTrueFalse)
        {
            _virtual = aTrueFalse;
        }
    private:
        void setNull();
        void constructProperties();

    };	// END of class OrientationSensor

} // end of namespace OpenSim


#endif // __OrientationSensor_h__


