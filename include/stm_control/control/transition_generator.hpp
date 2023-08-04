#pragma once

#include "bezier.h"
#include "stm_control/gaits/transition_gait.hpp"
#include <chrono>
#include "nlopt.hpp"
#include <vector>
#include "stm_control/trapezoidal_traj.h"
#include "stm_control/control/rover_trajectory.hpp"

namespace TransitionGenerator {

    /**
     * @brief returns a TransitionGait that executes a bezier curve to move between gait1 and gait2
     * @param gait1 the gait to start from
     * @param gait2 the gait to switch to
     * @param t1 the time 
    */
    TransitionGait generate(TransitionType type, Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);

    /**
     * @brief returns a trapezoidal trajectory from the current position to the neutral position (all joints at 0)
    */
    std::vector<TrapezoidalTrajectory::TrapTraj<int64_t>> findNaiveTrajectory(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);
        /**
     * @brief returns a cubic Bezier curve that moves from gait1 to gait2
    */
    Bezier::Curve<int64_t> findOptimalCubicCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);
        /**
     * @brief returns a spline that moves from gait1 to gait2
    */
    Bezier::Spline<int64_t> findOptimalWaypointCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);
        /**
     * @brief returns a cubic linear spline that moves from gait1 to gait2
    */
    Bezier::Spline<int64_t> findOptimalWaypointLinear(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);
}
