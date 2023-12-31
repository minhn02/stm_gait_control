#include "stm_control/control/state_machine.hpp"

#include "stm_control/gaits/idle_gait.hpp"
#include "stm_control/gaits/squirming_gait.hpp"
#include "stm_control/gaits/transition_gait.hpp"
#include "stm_control/gaits/wheelwalking_gait.hpp"
#include "stm_control/gaits/command_position_gait.hpp"
#include "stm_control/gaits/initial_gait.hpp"

namespace StateMachine {

    SquirmingGait squirmingGait = SquirmingGait();
    IdleGait idleGait = IdleGait();
    TransitionGait transition = TransitionGait();
    WheelWalkingGait wheelWalkingGait = WheelWalkingGait();
    CommandPositionGait commandPositionGait = CommandPositionGait();
    InitialGait initialGait = InitialGait();

    StateMachine::StateMachine(std::chrono::nanoseconds startTime) {
        startTime_ = startTime;
        currState_ = &initialGait;
        prevState_ = &initialGait;
        transitionStarted_ = false;
        transitionType_ = TransitionType::NAIVE;
        inStartup_ = true;
        madeCommandGait_ = false;
    }

    void StateMachine::switchState(uint8_t nextState, std::chrono::nanoseconds currTime) {
        if (!transitionStarted_) {
            if (nextState == 0) {
                currState_ = &idleGait;
            } else if (nextState == 1) {
                currState_ = &squirmingGait;
            } else if (nextState == 2) {
                currState_ = &wheelWalkingGait;
            } else if (nextState == 3) {
                transitionType_ = TransitionType::NAIVE;
            } else if (nextState == 4) {
                transitionType_ = TransitionType::BEZIER;
            } else if (nextState == 5) {
                transitionType_ = TransitionType::LINEAR_WAYPOINT;
            } else if (nextState == 6) {
                transitionType_ = TransitionType::BEZIER_WAYPOINT;
            }
        }
    }

    std::map<Joint, double> StateMachine::execute(std::chrono::nanoseconds time, std::map<Joint, double> currPositions) {
        std::map<Joint, double> jointCommands;
        
        // handle state transition
        if (currState_ != prevState_ && prevState_ != &idleGait && !inStartup_) {
            // std::cout << "in state transition: " << "transition started: " << transitionStarted_ << " currState: " << currState_ << " prevState: " << prevState_ << std::endl;
            if (transitionStarted_) {
                // std::cout << "transition started, running transition" << std::endl;
                if (!transition.isFinished(time)) {
                    // std::cout << "transition not finished, running transition" << std::endl;
                    return transition.run(time, startTime_, currPositions);
                } else {
                    std::cout << "transition finished, starting gait at: " << (time+t2_).count() << std::endl;
                    transitionStarted_ = false;
                    startTime_ = time - t2_;
                    prevState_ = currState_;
                    return currState_->run(time, startTime_, currPositions);
                }
            } else {
                transition = TransitionGenerator::generate(transitionType_, prevState_, currState_, startTime_, time, &tt_, &t2_);
                std::cout << "Generated transition: t_t: " << tt_.count() << " t_2: " << t2_.count() << std::endl;
                transitionStarted_ = true;
                startTime_ = time;
                return transition.run(time, startTime_, currPositions);
            }
        } else {
            if (inStartup_) {
                // move to idle gait
                if (!madeCommandGait_) {
                    startTime_ = time;
                    commandPositionGait = CommandPositionGait(time.count(), currPositions[Joint::STEERING_JOINT], currPositions[Joint::BOGIE_JOINT], -40 * (M_PI / 180.0), 0);
                    currState_ = &commandPositionGait;
                    madeCommandGait_ = true;
                }

                if (commandPositionGait.isFinished(time)) {
                    std::cout << "finished startup" << std::endl;
                    prevState_ = &idleGait;
                    currState_ = &idleGait;
                    startTime_ = time;
                    inStartup_ = false;
                }
            } else if (prevState_ == &idleGait) {
                startTime_ = time;
                prevState_ = currState_;
            }
            return currState_->run(time, startTime_, currPositions);
        }
    }

    uint8_t StateMachine::getCurrState() {
        if (currState_ == &idleGait) {
            return 0;
        } else if (currState_ == &squirmingGait) {
            return 1;
        } else if (currState_ == &wheelWalkingGait) {
            return 2;
        } else if (currState_ == &transition) {
            return 3;
        } else if (currState_ == &commandPositionGait) {
            return 4;
        } else if (currState_ == &initialGait) {
            return 5;
        } else {
            return 0;
        }
    }

    bool StateMachine::inTransition() {
        return transitionStarted_;
    }

    int64_t StateMachine::getCurrGaitTime(std::chrono::nanoseconds time) {
        if (currState_->getPeriod().count() == 0) {
            return 0;
        } else {
            return (time.count() - startTime_.count()) % currState_->getPeriod().count();
        }
    }
}

PYBIND11_MODULE(stm_state_machine, m) {
    py::enum_<Joint>(m, "Joint")
        .value("STEERING_JOINT", Joint::STEERING_JOINT)
        .value("STEERING_JOINT_VEL", Joint::STEERING_JOINT_VEL)
        .value("BOGIE_JOINT_VEL", Joint::BOGIE_JOINT_VEL)
        .value("BOGIE_JOINT", Joint::BOGIE_JOINT)
        .value("FRONT_LEFT_WHEEL", Joint::FRONT_LEFT_WHEEL)
        .value("FRONT_RIGHT_WHEEL", Joint::FRONT_RIGHT_WHEEL)
        .value("BACK_LEFT_WHEEL", Joint::BACK_LEFT_WHEEL)
        .value("BACK_RIGHT_WHEEL", Joint::BACK_RIGHT_WHEEL)
        .value("TRANSITIONING", Joint::TRANSITIONING);

    py::class_<StateMachine::StateMachine>(m, "StateMachine")
        .def(py::init<std::chrono::nanoseconds>())
        .def("switchState", &StateMachine::StateMachine::switchState)
        .def("execute", &StateMachine::StateMachine::execute)
        .def("getCurrState", &StateMachine::StateMachine::getCurrState)
        .def("inTransition", &StateMachine::StateMachine::inTransition)
        .def("getCurrGaitTime", &StateMachine::StateMachine::getCurrGaitTime);
}
