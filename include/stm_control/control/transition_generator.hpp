#pragma once

#include "bezier.h"
#include "stm_control/gaits/transition_gait.hpp"
#include <chrono>
#include "nlopt.hpp"
#include <vector>

namespace TransitionGenerator {

    /**
     * @brief returns a TransitionGait that executes a bezier curve to move between gait1 and gait2
     * @param gait1 the gait to start from
     * @param gait2 the gait to switch to
     * @param t1 the time 
    */
    TransitionGait generate(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);


    /**
     * @brief returns the cubic curve that moves from gait1 to gait2
    */
    Bezier::Curve<long> findOptimalCubicCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2);
}
