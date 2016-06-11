#ifndef OPENSIM_ORIENTATIONSENSOR_H_
#define OPENSIM_ORIENTATIONSENSOR_H_



// INCLUDE
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


    //=============================================================================
    //=============================================================================
    /**
    * A class implementing an OrientationSensor.
    *
    * @author Peter Loan
    * @version 1.0
    */
    class OSIMEXTENDEDIK_API OrientationSensor : public Object {
        OpenSim_DECLARE_CONCRETE_OBJECT(OrientationSensor, Object);
        OpenSim_DECLARE_PROPERTY(fixed, bool,
            "Flag (true or false) specifying whether or not a marker "
        "should be kept fixed in the marker placement step.  i.e. If false, the marker is allowed to move.");
        OpenSim_DECLARE_PROPERTY(body, std::string,
            "Body segment in the model on which the orientation sensor resides.");
        OpenSim_DECLARE_PROPERTY(offset, SimTK::Vec3,
            "Location of the orientation sensor on the body segment.");
        OpenSim_DECLARE_PROPERTY(rotation, SimTK::Vec3,
            "Rotation of the orientation sensor frame with respect to the body one.");

        class Body;

        //=============================================================================
        // DATA
        //=============================================================================
    private:

    protected:
        const Model* _model;

        // Body that the marker is attached to
        OpenSim::Body* _body;

        // Support for Display
        VisibleObject _displayer;

        /** A temporary kluge until the default mechanism is working */
        static Geometry *_defaultGeometry;
        bool _virtual;

        //=============================================================================
        // METHODS
        //=============================================================================
        //--------------------------------------------------------------------------
        // CONSTRUCTION
        //--------------------------------------------------------------------------
    public:
        OrientationSensor();
        OrientationSensor(const OrientationSensor &aOSensor);
        virtual ~OrientationSensor();

        static void deleteOSensor(OrientationSensor* aOSensor) { if (aOSensor) delete aOSensor; }

#ifndef SWIG
        OrientationSensor& operator=(const OrientationSensor &aOSensor);
#endif
        void copyData(const OrientationSensor &aOSensor);

        virtual void updateFromOSensor(const OrientationSensor &aOSensor);
        virtual OpenSim::Body& getBody() const { return *_body; }
        virtual void changeBody(OpenSim::Body& aBody);
        virtual void changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody);
        virtual void scale(const SimTK::Vec3& aScaleFactors);
        virtual void connectOSensorToModel(const Model& aModel);
        virtual void updateGeometry();

        virtual const VisibleObject* getDisplayer() const { return &_displayer; }
        virtual VisibleObject*	updDisplayer() { return &_displayer; };

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
        //=============================================================================
    };	// END of class OrientationSensor
    //=============================================================================
    //=============================================================================

} // end of namespace OpenSim


#endif // OPENSIM_ORIENTATIONSENSOR_H_


