#include "stm_control/gaits/idle_gait.hpp"


Eigen::VectorXd IdleGait::evaluate(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << 0, 0;
    return result;
}

Eigen::VectorXd IdleGait::derivative(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << 0, 0;
    return result;
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

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)}};
    return publishMap;
}