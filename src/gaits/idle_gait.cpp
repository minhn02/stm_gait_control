#include "stm_control/gaits/idle_gait.hpp"


Eigen::VectorXd IdleGait::evaluate(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << -steeringLimit_, 0;
    return result;
}

Eigen::VectorXd IdleGait::derivative(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << 0, 0;
    return result;
}

Eigen::Vector3d IdleGait::displacement(std::chrono::nanoseconds t) {
    return Eigen::Vector3d::Zero();
}

bool IdleGait::isFinished(std::chrono::nanoseconds currTime) {
    return false;
}
    
std::map<Joint, double> IdleGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    std::chrono::nanoseconds gaitTime = time - startTime;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(gaitTime);
    Eigen::VectorXd velocities = derivative(gaitTime);

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, 0.0}, {Joint::FRONT_RIGHT_WHEEL, 0.0},
                                          {Joint::BACK_LEFT_WHEEL, 0.0}, {Joint::BACK_RIGHT_WHEEL, 0.0},
                                          {Joint::TRANSITIONING, 0.0}};
    return publishMap;
}