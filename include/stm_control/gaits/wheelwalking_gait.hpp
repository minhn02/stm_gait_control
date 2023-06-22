#pragma once

#include "stm_control/gaits/gait.hpp"

class WheelWalkingGait : public Gait::Gait {

    public:
    WheelWalkingGait(double steeringAmplitude = 0.5, double bogieAmplitude = 0.1, long period = 3e9);

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
    double steeringAmplitude_;
    double bogieAmplitude_;
    long period_;
};
