#include "stm_control/gaits/transition_gait.hpp"

TransitionGait::TransitionGait() {
    curve_ = Bezier::Curve<int64_t>();
}

TransitionGait::TransitionGait(Bezier::Curve<int64_t> curve) {
    curve_ = curve;
    startTime_ = std::chrono::nanoseconds(curve.getT0());
    period_ = curve.getT();
}

Eigen::VectorXd TransitionGait::evaluate(std::chrono::nanoseconds t) {
    return curve_.evaluate(t.count());
}

Eigen::VectorXd TransitionGait::derivative(std::chrono::nanoseconds t) {
    return curve_.dEvaluate(t.count());
}

bool TransitionGait::isFinished(std::chrono::nanoseconds currTime) {
    return (currTime - startTime_).count() > curve_.getT();
}

std::map<Joint, double> TransitionGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    // std::chrono::nanoseconds gaitTime = time - startTime;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(time);
    Eigen::VectorXd velocities = derivative(time);

    // calculate wheel speeds
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]}};
    return publishMap;
}