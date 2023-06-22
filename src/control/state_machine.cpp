#include "stm_control/control/state_machine.hpp"

#include "stm_control/gaits/idle_gait.hpp"
#include "stm_control/gaits/squirming_gait.hpp"
#include "stm_control/gaits/transition_gait.hpp"
#include "stm_control/gaits/wheelwalking_gait.hpp"

namespace StateMachine {

    SquirmingGait squirmingGait = SquirmingGait();
    IdleGait idleGait = IdleGait();
    TransitionGait transition = TransitionGait();
    WheelWalkingGait wheelWalkingGait = WheelWalkingGait();

    StateMachine::StateMachine(std::chrono::nanoseconds startTime) {
        startTime_ = startTime;
        currState_ = &idleGait;
        prevState_ = &idleGait;
        transitionStarted_ = false;
    }

    void StateMachine::switchState(uint8_t nextState, std::chrono::nanoseconds currTime) {
        if (!transitionStarted_) {
            if (nextState == 0) {
                currState_ = &idleGait;
            } else if (nextState == 1) {
                currState_ = &squirmingGait;
            } else if (nextState == 2) {
                currState_ = &wheelWalkingGait;
            }
        }
    }

    std::map<Joint, double> StateMachine::execute(std::chrono::nanoseconds time, std::map<Joint, double> currPositions) {
        std::map<Joint, double> jointCommands;
        
        // handle state transition
        if (currState_ != prevState_ && prevState_ != &idleGait) {
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
                transition = TransitionGenerator::generate(prevState_, currState_, startTime_, time, &tt_, &t2_);
                std::cout << "Generated transition: t_t: " << tt_.count() << " t_2: " << t2_.count() << std::endl;
                transitionStarted_ = true;
                startTime_ = time;
                return transition.run(time, startTime_, currPositions);
            }
        } else {
            if (prevState_ == &idleGait) {
                startTime_ = time;
                prevState_ = currState_;
            }
            return currState_->run(time, startTime_, currPositions);
        }
    }
}

PYBIND11_MODULE(stm_state_machine, m) {
    py::enum_<Joint>(m, "Joint")
        .value("STEERING_JOINT", Joint::STEERING_JOINT)
        .value("BOGIE_JOINT", Joint::BOGIE_JOINT)
        .value("FRONT_LEFT_WHEEL", Joint::FRONT_LEFT_WHEEL)
        .value("FRONT_RIGHT_WHEEL", Joint::FRONT_RIGHT_WHEEL)
        .value("BACK_LEFT_WHEEL", Joint::BACK_LEFT_WHEEL)
        .value("BACK_RIGHT_WHEEL", Joint::BACK_RIGHT_WHEEL);

    py::class_<StateMachine::StateMachine>(m, "StateMachine")
        .def(py::init<std::chrono::nanoseconds>())
        .def("switchState", &StateMachine::StateMachine::switchState)
        .def("execute", &StateMachine::StateMachine::execute);
}
