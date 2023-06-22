#pragma once

#include "Eigen/Dense"
#include <vector>
#include <cmath>
#include <algorithm>

namespace WheelController {
    const double LX = 200;
    const double LY = 200;
    const double WHEEL_RADIUS = 100;

    /**
     * @brief Calculates the wheel speeds for a given steering angle and velocity to drive straight
     * 
     * From Crawling Locomotion Enabled by a Novel Actuated Rover Chassis (fig 2-5)
     * 
     * @param velocity the speed for the rover to move at
     * @param steeringAngle The steering angle in radians
     * @param steeringVelocity The steering velocity in radians per second
     * 
     * @returns A vector of wheel speeds in radians per second
    */
    std::vector<double> calculateWheelSpeeds(double velocity, double steeringAngle, double steeringVelocity);

}