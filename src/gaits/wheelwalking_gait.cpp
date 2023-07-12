#include "stm_control/gaits/wheelwalking_gait.hpp"

WheelWalkingGait::WheelWalkingGait(){
    steerTransientDuration_ = 2e8;
    steeringLimit_ = 40 * (M_PI / 180.0);
    steering_velocity_ = 15 * (M_PI / 180.0) / 1e9;

    bogieTransientDuration_ = 5e8;
    bogieLimit_ = 25 * (M_PI / 180.0);
    bogie_velocity_ = 10 * (M_PI / 180.0) / 1e9;

    zeroToForwardBogieTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(0, bogieLimit_, bogie_velocity_, 0, bogieTransientDuration_);
    forwardSteerTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(-steeringLimit_, steeringLimit_, steering_velocity_, zeroToForwardBogieTraj_.duration(), steerTransientDuration_);
    forwardtoZeroBogieTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(bogieLimit_, 0, bogie_velocity_, zeroToForwardBogieTraj_.duration() + forwardSteerTraj_.duration(), bogieTransientDuration_);
    zerotoBackwardBogieTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(0, -bogieLimit_, bogie_velocity_, zeroToForwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration(), bogieTransientDuration_);
    backwardSteerTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(steeringLimit_, -steeringLimit_, steering_velocity_, zeroToForwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration(), steerTransientDuration_);
    backwardtoZeroBogieTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(-bogieLimit_, 0, bogie_velocity_, zeroToForwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration() + backwardSteerTraj_.duration(), bogieTransientDuration_);

    period_ = zeroToForwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration() + backwardSteerTraj_.duration() + backwardtoZeroBogieTraj_.duration();
}

Eigen::VectorXd WheelWalkingGait::evaluate(std::chrono::nanoseconds t) {
    int64_t state_time = t.count() % period_;
    Eigen::VectorXd result(2);
    if (state_time < zeroToForwardBogieTraj_.duration()) {
        result << -steeringLimit_, zeroToForwardBogieTraj_.position(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration()) {
        result << forwardSteerTraj_.position(state_time), bogieLimit_;
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration()) {
        result << steeringLimit_, forwardtoZeroBogieTraj_.position(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration()) {
        result << steeringLimit_, zerotoBackwardBogieTraj_.position(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration() + backwardSteerTraj_.duration()) {
        result << backwardSteerTraj_.position(state_time), -bogieLimit_;
    } else {
        result << -steeringLimit_, backwardtoZeroBogieTraj_.position(state_time);
    }
    return result;
}

Eigen::VectorXd WheelWalkingGait::derivative(std::chrono::nanoseconds t) {
    int64_t state_time = t.count() % period_;
    Eigen::VectorXd result(2);
    if (state_time < zeroToForwardBogieTraj_.duration()) {
        result << 0, zeroToForwardBogieTraj_.velocity(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration()) {
        result << forwardSteerTraj_.velocity(state_time), 0;
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration()) {
        result << 0, forwardtoZeroBogieTraj_.velocity(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration()) {
        result << 0, zerotoBackwardBogieTraj_.velocity(state_time);
    } else if (state_time < zerotoBackwardBogieTraj_.duration() + forwardSteerTraj_.duration() + forwardtoZeroBogieTraj_.duration() + zerotoBackwardBogieTraj_.duration() + backwardSteerTraj_.duration()) {
        result << backwardSteerTraj_.velocity(state_time), 0;
    } else {
        result << 0, backwardtoZeroBogieTraj_.velocity(state_time);
    }
    return result;
}

bool WheelWalkingGait::isFinished(std::chrono::nanoseconds currTime) {
    return false;
}
    
std::map<Joint, double> WheelWalkingGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    std::chrono::nanoseconds gaitTime = time - startTime;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(gaitTime);
    Eigen::VectorXd velocities = derivative(gaitTime);

    // calculate wheel speeds
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]},
                                          {Joint::TRANSITIONING, 0.0}};
    return publishMap;
}