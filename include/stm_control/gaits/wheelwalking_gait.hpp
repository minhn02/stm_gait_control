#pragma once

#include "stm_control/gaits/gait.hpp"
#include "stm_control/trapezoidal_traj.h"

class WheelWalkingGait : public Gait::Gait {

    public:
    WheelWalkingGait();

    Eigen::VectorXd evaluate(std::chrono::nanoseconds t);

    /**
     * Get derivative/velocity of controlled joints at time t
     * @param t the time to evaluate the joint velocities at
     *
     * @returns the velocity of controlled joints at time t
     */
    Eigen::VectorXd derivative(std::chrono::nanoseconds t);

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
    int64_t period_;

    int64_t steerTransientDuration_;
    double steeringLimit_;
    double steering_velocity_;
    TrapezoidalTrajectory::TrapTraj<int64_t> forwardSteerTraj_;
    TrapezoidalTrajectory::TrapTraj<int64_t> backwardSteerTraj_;

    int64_t bogieTransientDuration_;
    double bogieLimit_;
    double bogie_velocity_;
    TrapezoidalTrajectory::TrapTraj<int64_t> forwardtoZeroBogieTraj_;
    TrapezoidalTrajectory::TrapTraj<int64_t> zeroToForwardBogieTraj_;
    TrapezoidalTrajectory::TrapTraj<int64_t> backwardtoZeroBogieTraj_;
    TrapezoidalTrajectory::TrapTraj<int64_t> zerotoBackwardBogieTraj_;
};
