#pragma once

#include "Eigen/Dense"
#include "stm_control/joints.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <iterator>
#include <iostream>
#include <map>
#include <string>
#include <chrono>
#include "bezier.h"
#include "stm_control/control/wheel_controller.hpp"

using namespace std::literals::chrono_literals;

namespace Gait {
    class Gait {
        public:
        /**
         * Get position of controlled joints at time t
         * @param p0 the intial position the joints start at
         * @param t the time to evaluate the joint positions at
         * 
         * @returns the positon of controlled joints at time t
        */
        virtual Eigen::VectorXd evaluate(std::chrono::nanoseconds t) = 0;

        /**
         * Get derivative/velocity of controlled joints at time t
         * @param t the time to evaluate the joint velocities at
         * 
         * @returns the velocity of controlled joints at time t
        */
        virtual Eigen::VectorXd derivative(std::chrono::nanoseconds t) = 0;

        /**
         * @returns if the gait is finished (for non-periodic gaits)
        */
        virtual bool isFinished(std::chrono::nanoseconds currTime) = 0;
        
        /**
         * Given a ROS time, publish joint messages to control the joints according to the gait movement
         * @param time the current ROS time
         * @param startTime the time the gait began executing
         * @param jointStates information about the current joint positions
         * 
         * @returns map of {topic names, values} to be published to controllers
        */
        virtual std::map<Joint, double> run(std::chrono::nanoseconds time, std::chrono::nanoseconds startTime, std::map<Joint, double> jointStates) = 0;

        /**
         * @returns the period of the gait
        */ 
        virtual std::chrono::nanoseconds getPeriod() = 0;
    };
}
