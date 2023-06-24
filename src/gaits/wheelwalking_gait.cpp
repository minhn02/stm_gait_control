#include "stm_control/gaits/wheelwalking_gait.hpp"

WheelWalkingGait::WheelWalkingGait(double steeringAmplitude, double bogieAmplitude, int64_t period){
    steeringAmplitude_ = steeringAmplitude;
    bogieAmplitude_ = bogieAmplitude;
    period_ = period;
}

Eigen::VectorXd WheelWalkingGait::evaluate(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << steeringAmplitude_*std::sin((t.count()*2*M_PI)/(period_)), bogieAmplitude_*std::sin((t.count()*2*M_PI)/(period_));
    return result;
}

Eigen::VectorXd WheelWalkingGait::derivative(std::chrono::nanoseconds t) {
    Eigen::VectorXd result(2);
    result << steeringAmplitude_*(2*M_PI)/period_ * std::cos((t.count()*2*M_PI)/(period_)), bogieAmplitude_*(2*M_PI)/period_ * std::cos((t.count()*2*M_PI)/(period_));
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
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(100, positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]}};
    return publishMap;
}