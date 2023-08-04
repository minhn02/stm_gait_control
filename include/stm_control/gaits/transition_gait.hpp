#pragma once

#include "stm_control/gaits/gait.hpp"
#include "bezier.h"
#include "stm_control/trapezoidal_traj.h"
#include "algorithm"
#include "stm_control/control/rover_trajectory.hpp"

enum TransitionType {
    NAIVE,
    LINEAR_WAYPOINT,
    BEZIER_WAYPOINT,
    BEZIER,
};

class TransitionGait : public Gait::Gait {
public:

    TransitionGait();
    TransitionGait(Bezier::Curve<int64_t> curve);
    TransitionGait(Bezier::Spline<int64_t> spline);
    TransitionGait(std::vector<TrapezoidalTrajectory::TrapTraj<int64_t>> trapTrajectories);

    Eigen::VectorXd evaluate(std::chrono::nanoseconds t);

    /**
     * Get derivative/velocity of controlled joints at time t
     * @param t the time to evaluate the joint velocities at
     *
     * @returns the velocity of controlled joints at time t
     */
    Eigen::VectorXd derivative(std::chrono::nanoseconds t);

    Eigen::Vector3d displacement(std::chrono::nanoseconds t);

    /**
     * @returns if the gait is finished (for non-periodic gaits)
     */
    bool isFinished(std::chrono::nanoseconds currTime);

    /**
     * Given a ROS time, publish joint messages to control the joints according to the gait movement
     * @param time the current ROS time
     * @param startTime the time the gait began executing
     * @param jointStates information about the current joint positions
     *
     * @returns map of {joints, values} to be published to controllers
     */
    std::map<Joint, double> run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates);


    std::chrono::nanoseconds getPeriod() {
        return std::chrono::nanoseconds(period_);
    }

    private:
    TransitionType type_;
    Bezier::Curve<int64_t> curve_;
    Bezier::Spline<int64_t> spline_;
    std::vector<TrapezoidalTrajectory::TrapTraj<int64_t>> trapTrajectories_;
    std::chrono::nanoseconds startTime_;
    int64_t period_;
};