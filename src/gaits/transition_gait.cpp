#include "stm_control/gaits/transition_gait.hpp"

TransitionGait::TransitionGait() {
}

TransitionGait::TransitionGait(Bezier::Curve<int64_t> curve) {
    type_ = TransitionType::BEZIER;
    curve_ = curve;
    startTime_ = std::chrono::nanoseconds(curve.getT0());
    period_ = curve.getT();
}

TransitionGait::TransitionGait(Bezier::Spline<int64_t> spline) {
    type_ = TransitionType::BEZIER_WAYPOINT;
    spline_ = spline;
    startTime_ = std::chrono::nanoseconds(spline.getT0());
    period_ = spline.getT();
}

TransitionGait::TransitionGait(std::vector<TrapezoidalTrajectory::TrapTraj<int64_t>> trapTrajectories) {
    type_ = TransitionType::NAIVE;
    trapTrajectories_ = trapTrajectories;
    startTime_ = std::chrono::nanoseconds(trapTrajectories[0].getT0());
    int64_t maxPeriod = 0;
    for (std::size_t i = 0; i < trapTrajectories.size(); i++) {
        maxPeriod = std::max(maxPeriod, trapTrajectories[i].duration());
    }
    period_ = maxPeriod;
}

Eigen::VectorXd TransitionGait::evaluate(std::chrono::nanoseconds t) {
    VectorXd point(trapTrajectories_.size());
    switch (type_) {
        case NAIVE:
            for (unsigned i = 0; i < trapTrajectories_.size(); i++) {
                point(i) = trapTrajectories_[i].position(t.count());
            }
            return point;
        case BEZIER:
            return curve_.evaluate(t.count());
        case BEZIER_WAYPOINT:
            return spline_.evaluate(t.count());
        default:
            return Eigen::VectorXd::Zero(1);
    }
}

Eigen::VectorXd TransitionGait::derivative(std::chrono::nanoseconds t) {
    VectorXd point(trapTrajectories_.size());
    switch (type_) {
        case NAIVE:
            for (std::size_t i = 0; i < trapTrajectories_.size(); i++) {
                point(i) = trapTrajectories_[i].velocity(t.count());
            }
            return point;
        case BEZIER:
            return curve_.dEvaluate(t.count());
        case BEZIER_WAYPOINT:
            return spline_.dEvaluate(t.count());
        default:
            return Eigen::VectorXd::Zero(1);
    }
}

Eigen::Vector3d TransitionGait::displacement(std::chrono::nanoseconds t) {
    return Eigen::Vector3d::Zero();
}

bool TransitionGait::isFinished(std::chrono::nanoseconds currTime) {
    switch (type_) {
        case NAIVE:
            return currTime.count() > startTime_.count() + period_;
        case BEZIER:
            return curve_.isFinished(currTime.count());
        case BEZIER_WAYPOINT:
            return spline_.isFinished(currTime.count());
        default:
            return true;
    }
}

std::map<Joint, double> TransitionGait::run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) {
    // process ROS time and system state
    // std::chrono::nanoseconds gaitTime = time - startTime;

    Eigen::VectorXd p0(2);
    p0 << jointStates[Joint::STEERING_JOINT], jointStates[Joint::BOGIE_JOINT];

    // evaluate positions
    Eigen::VectorXd positions = evaluate(time);
    Eigen::VectorXd velocities = derivative(time)*1e9;

    // calculate wheel speeds
    std::vector<double> wheelSpeeds = WheelController::calculateWheelSpeeds(positions(0), velocities(0));    

    std::map<Joint, double> publishMap = {{Joint::STEERING_JOINT, positions(0)}, {Joint::BOGIE_JOINT, positions(1)},
                                          {Joint::STEERING_JOINT_VEL, velocities(0)}, {Joint::BOGIE_JOINT_VEL, velocities(1)},
                                          {Joint::FRONT_LEFT_WHEEL, wheelSpeeds[0]}, {Joint::FRONT_RIGHT_WHEEL, wheelSpeeds[1]},
                                          {Joint::BACK_LEFT_WHEEL, wheelSpeeds[2]}, {Joint::BACK_RIGHT_WHEEL, wheelSpeeds[3]},
                                          {Joint::TRANSITIONING, 1.0}};
    return publishMap;
}