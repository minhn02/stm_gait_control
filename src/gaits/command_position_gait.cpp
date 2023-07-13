#include "stm_control/gaits/command_position_gait.hpp"

CommandPositionGait::CommandPositionGait() {}

CommandPositionGait::CommandPositionGait(int64_t t0, double currSteering, double currBogie, double desiredSteering, double desiredBogie) {
    steerTransientDuration_ = 2e8;
    bogieTransientDuration_ = 5e8;
    steering_velocity_ = 15 * (M_PI / 180.0) / 1e9;
    bogie_velocity_ = 10 * (M_PI / 180.0) / 1e9;

    steeringTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(currSteering, desiredSteering, steering_velocity_, t0, steerTransientDuration_);
    bogieTraj_ = TrapezoidalTrajectory::TrapTraj<int64_t>(currBogie, desiredBogie, bogie_velocity_, t0, bogieTransientDuration_);

    std::cout << "currSteering: " << currSteering << " desiredSteering: " << desiredSteering << std::endl;
    std::cout << "currBogie: " << currBogie << " desiredBogie: " << desiredBogie << std::endl;

    period_ = std::max(steeringTraj_.duration(), bogieTraj_.duration());
    isFinished_ = false;
}

Eigen::VectorXd CommandPositionGait::evaluate(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << steeringTraj_.position(t.count()), bogieTraj_.position(t.count());
    return result;
}

Eigen::VectorXd CommandPositionGait::derivative(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << steeringTraj_.velocity(t.count()), bogieTraj_.velocity(t.count());
    return result;
}

bool CommandPositionGait::isFinished(std::chrono::nanoseconds currTime) {
    return isFinished_;
}
    
std::map<Joint, double> CommandPositionGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    std::chrono::nanoseconds gaitTime = time - startTime;

    isFinished_ = gaitTime.count() > period_;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(time);
    Eigen::VectorXd velocities = derivative(time)*1e9;

    std::cout << "positions: " << positions(0) << " velocities: " << velocities(0) << std::endl;

    // calculate wheel speeds
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]},
                                          {Joint::TRANSITIONING, 0.0}};
                                          
    return publishMap;
}