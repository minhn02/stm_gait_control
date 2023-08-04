#pragma once

#include "bezier.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>

#include "stm_control/gaits/gait.hpp"


namespace RoverTrajectory {

    std::vector<std::vector<double>> transform_joint_movement(std::vector<double> beta_list, double dt, std::vector<double> initial_position, double initial_beta);

    Vector3d find_closest_waypoint(Vector3d curr_position, Vector3d waypoint);

    /***
        * @brief Returns the displacement of a rover over time
        * @param func a function that returns the velocity of the rover at time t
        * @param num_points the number of points to sample the function at
        * @param t0 the start time of the trajectory
        * @param duration the duration of the trajectory
        * @param initial_position the initial position of the rover in [X, Y, Theta] space
        * @param initial_beta the initial position of the steering joint
        * @return a vector of [X, Y, Theta] at the end of the trajectory
    */
    std::vector<double> calculate_rover_displacement(std::vector<double> beta_list, double dt, std::vector<double> initial_position, double initial_beta);

    /***
        * @brief Returns the displacement of a rover following a gait trajectory
        * @param func a gait which returns the joint positions at time t
        * @param num_points the number of points to sample the function at
        * @param t0 the start time of the trajectory
        * @param duration the duration of the trajectory
        * @param initial_position the initial position of the rover in [X, Y, Theta] space
        * @return a vector of [X, Y, Theta] at the end of the trajectory
    */
    Bezier::Spline<int64_t> translate_to_cartesian_gait(Gait::Gait* gait, size_t num_points, int64_t period, int64_t t0, std::vector<double> initial_position);
}
