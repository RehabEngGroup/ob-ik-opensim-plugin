/* -------------------------------------------------------------------------- *
 *   Orientation Based Inverse Kinematics : InverseKinematicExtendedSolver.h  *
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

#ifndef OPENSIM_INVERSE_KINEMATICS_EXTENDED_SOLVER_H_
#define OPENSIM_INVERSE_KINEMATICS_EXTENDED_SOLVER_H_

#include "OpenSim/Simulation/AssemblySolver.h"
#include "OpenSim/Simulation/osimExtendedIKDLL.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim {

class CoordinateReference;
class MarkersReference;
class OrientationSensorsReference;

//=============================================================================
//=============================================================================
/**
 * Solve for the coordinates (degrees-of-freedom) of the model that satisfy the
 * set of constraints imposed on the model as well as set of desired coordinate
 * values.  The InverseKinematics provides the option to convert the problem to
 * an approximate one where the constraint violations are treated as penalties
 * to be minimized rather than strictly enforced. This can speedup the solution
 * time and can be used to seed the constrained problem closer to the solution.
 *
 * The InverseKinematics objective:
 * \f[
 *   min: J = sum(Wm_i*(m_i-md_i)^T*(m_i-md_i)) + sum(Wq_j*(q_j-qd_j)^2)) +
 *            [Wc*sum(c_{err})^2]
 * \f]
 * where m_i and md_i are the model and desired marker locations (Vec3); q_j
 * and qd_j are model and desired joint coordinates. Wm_i and Wq_j are the
 * marker and coordinate weightings, respectively, and Wc is the weighting on
 * constraint errors. When Wc == Infinity, the second term is not included,
 * but instead q is subject to the constraint equations:
 *      \f[ c_{err} = G(q)-Go = 0 \f]
 *
 * When the model (and the number of goals) is guaranteed not to change and
 * the initial state is close to the InverseKinematics solution (i.e. from the
 * initial assemble()), then track() is an efficient method for updating the
 * configuration to determine the small change in coordinate values, q.
 *
 * See SimTK::Assembler for more algorithmic details of the underlying solver.
 *
 * @author Ajay Seth
 */
class OSIMEXTENDEDIK_API InverseKinematicsExtendedSolver : public AssemblySolver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:

    // The marker reference values and weightings
    MarkersReference &_markersReference;

    // Markers collectively form a single assembly condition for the SimTK::Assembler
    SimTK::Markers *_markerAssemblyCondition;

    // The OSensor reference values and weightings
    OrientationSensorsReference &_orientationSensorsReference;

    // OSensors collectively form a single assembly condition for the SimTK::Assembler
    SimTK::OrientationSensors *_orientationSensorAssemblyCondition;


//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    virtual ~InverseKinematicsExtendedSolver() {}

    InverseKinematicsExtendedSolver(const Model &model,
                            MarkersReference &markersReference,
                            SimTK::Array_<CoordinateReference> &coordinateReferences,
                            double constraintWeight = SimTK::Infinity);

    InverseKinematicsExtendedSolver(const Model &model,
                            OrientationSensorsReference &oSensorsReference,
                            SimTK::Array_<CoordinateReference> &coordinateReferences,
                            double constraintWeight = SimTK::Infinity);

    InverseKinematicsExtendedSolver(const Model &model,
                            MarkersReference &markersReference,
                            OrientationSensorsReference &oSensorsReference,
                            SimTK::Array_<CoordinateReference> &coordinateReferences,
                            double constraintWeight = SimTK::Infinity);

    /** Assemble a model configuration that meets the InverseKinematics conditions
        (desired values and constraints) starting from an initial state that
        does not have to satisfy the constraints. */
    //virtual void assemble(SimTK::State &s);

    /** Obtain a model configuration that meets the InverseKinematics conditions
        (desired values and constraints) given a state that satisfies or
        is close to satisfying the constraints. Note there can be no change
        in the number of constraints or desired coordinates. Desired
        coordinate values can and should be updated between repeated calls
        to track a desired trajectory of coordinate values. */
    //virtual void track(SimTK::State &s);

    /** Change the weighting of a marker to take affect when assemble or track is called next.
        Update a marker's weight by name. */
    void updateMarkerWeight(const std::string &markerName, double value);
    /** Update a marker's weight by its index. */
    void updateMarkerWeight(int markerIndex, double value);
    /** Update all oSensors weights by order in the orientationSensorReference passed in to
        construct the solver. */
    void updateMarkerWeights(const SimTK::Array_<double> &weights);

    /** Change the weighting of a oSensor to take affect when assemble or track is called next.
    Update a oSensor's weight by name. */
    void updateOSensorWeight(const std::string &sensorName, double value);
    /** Update a oSensor's weight by its index. */
    void updateOSensorWeight(int sensorIndex, double value);
    /** Update all oSensors weights by order in the orientationSensorReference passed in to
    construct the solver. */
    void updateOSensorWeights(const SimTK::Array_<double> &weights);

    /** Compute and return the spatial location of a marker in ground. */
    SimTK::Vec3 computeCurrentMarkerLocation(const std::string &markerName);
    SimTK::Vec3 computeCurrentMarkerLocation(int markerIndex);

    //SimTK::Vec3 computeCurrentOSensorLocation(const std::string &sensorName); // TODO: check if useful
    SimTK::Quaternion computeCurrentOSensorOrientation(const std::string &sensorName);
    //SimTK::Vec3 computeCurrentOSensorLocation(int markerIndex); // TODO: check if useful
    SimTK::Quaternion computeCurrentOSensorOrientation(int markerIndex);

    /** Compute and return the spatial locations of all markers in ground. */
    void computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations);

    void computeCurrentOSensorOrientations(SimTK::Array_<SimTK::Quaternion> &sensorOrientations);

    /** Compute and return the distance error between model marker and observation. */
    double computeCurrentMarkerError(const std::string &markerName);
    double computeCurrentMarkerError(int markerIndex);

    double computeCurrentOSensorError(const std::string &sensorName);
    double computeCurrentOSensorError(int sensorIndex);

    /** Compute and return the distance errors between all model markers and their observations. */
    void computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors);

    void computeCurrentOSensorErrors(SimTK::Array_<double> &sensorErrors);

    /** Compute and return the squared-distance error between model marker and observation. 
        This is cheaper than calling the error and squaring it, since distance from norm-2 */
    double computeCurrentSquaredMarkerError(const std::string &markerName);
    double computeCurrentSquaredMarkerError(int markerIndex);

    /** Compute and return the distance errors between all model marker and observations. */
    void computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors);

    /** Marker locations and errors may be computed in an order that is different
        from tasks file or listed in the model. Return the corresponding marker
        name for an index in the list of marker locations/errors returned by the
        solver. */
    std::string getMarkerNameForIndex(int markerIndex) const;
    std::string getOSensorNameForIndex(int sensorIndex) const;
protected:
    /** Internal method to convert the CoordinateReferences into goals of the
        assembly solver. Subclasses can override to include other goals
        such as point of interest matching (Marker tracking). This method is
        automatically called by assemble. */
    virtual void setupGoals(SimTK::State &s) override;
    /** Internal method to update the time, reference values and/or their
        weights that define the goals, based on the passed in state */
    virtual void updateGoals(const SimTK::State &s) override;

private:
    // Non-accessible cache of the marker values to be matched at a given state
    SimTK::Array_<SimTK::Vec3> _markerValues;
    SimTK::Array_<SimTK::Quaternion> _orientationSensorValues;
    bool hasMarkersFile_ = false;
    bool hasOSensorsFile_ = false;

    OpenSim::MarkersReference* mrp_ = nullptr;
    OpenSim::MarkersReference& mrr_ = *mrp_;

    OpenSim::OrientationSensorsReference* osrp_ = nullptr;
    OpenSim::OrientationSensorsReference& osrr_ = *osrp_;

    void setupMarkerGoals(SimTK::State &s);
    void setupOSensorGoals(SimTK::State &s);

//=============================================================================
};  // END of class InverseKinematicsExtendedSolver
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_KINEMATICS_EXTENDED_SOLVER_H_
