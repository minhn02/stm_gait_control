#include "stm_control/gaits/squirming_gait.hpp"

SquirmingGait::SquirmingGait() {
    transientDuration_ = 2e8;
    steeringLimit_ = 40 * (M_PI / 180.0);
    steering_velocity_ = 15 * (M_PI / 180.0) / 1e9;
    forwardTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(-steeringLimit_, steeringLimit_, steering_velocity_, 0, transientDuration_);
    backwardTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(steeringLimit_, -steeringLimit_, steering_velocity_, forwardTraj_.duration(), transientDuration_);
    period_ = forwardTraj_.duration() + backwardTraj_.duration();
}

Eigen::VectorXd SquirmingGait::evaluate(std::chrono::nanoseconds t) {
    int64_t state_time = t.count() % period_;
    Eigen::VectorXd result(2);
    if (state_time < forwardTraj_.duration()) {
        result << forwardTraj_.position(state_time), 0;
    } else {
        result << backwardTraj_.position(state_time), 0;
    }
    return result;
}

Eigen::VectorXd SquirmingGait::derivative(std::chrono::nanoseconds t) {
    int64_t state_time = t.count() % period_;
    Eigen::VectorXd result(2);
    if (state_time < forwardTraj_.duration()) {
        result << forwardTraj_.velocity(state_time), 0;
    } else {
        result << backwardTraj_.velocity(state_time), 0;
    }
    return result;
}

bool SquirmingGait::isFinished(std::chrono::nanoseconds currTime) {
    return false;
}
    
std::map<Joint, double> SquirmingGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    std::chrono::nanoseconds gaitTime = time - startTime;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(gaitTime);
    Eigen::VectorXd velocities = derivative(gaitTime)*1e9;

    // std::cout << "positions: " << positions(0) << " velocities: " << velocities(0) << std::endl;

    // calculate wheel speeds
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]},
                                          {Joint::TRANSITIONING, 0.0}};
                                          
    return publishMap;
}