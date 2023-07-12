#pragma once

#include "stm_control/gaits/gait.hpp"
#include "stm_control/control/transition_generator.hpp"
#include <memory>
#include <chrono>
#include <iostream>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace std::chrono_literals;

namespace StateMachine {

class StateMachine {
    public:
    StateMachine(std::chrono::nanoseconds startTime = 0ns);
    /**
     * @brief Changes the current state to nextState
     * 
     * @param nextState state to switch to
     * 
    */
    void switchState(uint8_t nextState, std::chrono::nanoseconds currTime);

    /**
     * @brief Runs at 10 hz, passing joint commands according to the current
     * Movement. Handles state switching.
    */
    std::map<Joint, double> execute(std::chrono::nanoseconds time, std::map<Joint, double> currPositions);

    private:
   bool madeCommandGait_;
    bool inStartup_;
    TransitionType transitionType_;
    Gait::Gait *prevState_;
    Gait::Gait *currState_;
    std::chrono::nanoseconds startTime_;
    bool transitionStarted_;
    std::chrono::nanoseconds t2_;
    std::chrono::nanoseconds tt_;
};


}