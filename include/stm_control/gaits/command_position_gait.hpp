#pragma once

#include "stm_control/gaits/gait.hpp"
#include "stm_control/trapezoidal_traj.h"
#include "stm_control/control/rover_trajectory.hpp"

class CommandPositionGait : public Gait::Gait {
public:
    CommandPositionGait();

    CommandPositionGait(int64_t t0, double currSteering, double currBogie, double desiredSteering, double desiredBogie);

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
    int64_t period_ = 0;
    TrapezoidalTrajectory::TrapTraj<int64_t> steeringTraj_;
    TrapezoidalTrajectory::TrapTraj<int64_t> bogieTraj_;

    int64_t steerTransientDuration_;
    double steering_velocity_;

    int64_t bogieTransientDuration_;
    double bogie_velocity_;

    bool isFinished_;
};