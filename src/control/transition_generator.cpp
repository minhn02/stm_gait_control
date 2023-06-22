#include "stm_control/control/transition_generator.hpp"

using namespace nlopt;
using namespace Eigen;

namespace TransitionGenerator {

    struct OptData {
        Gait::Gait *gait1;
        Gait::Gait *gait2;
        long t1;
        long startTime;
    };

    TransitionGait generate(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        Bezier::Curve<long> curve = findOptimalCubicCurve(gait1, gait2, startTime, t1, t_t, t_2);
    //  //TODO seperate start time from constructor
        return TransitionGait(curve);
    }

    double cost_func(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
        OptData *data = (OptData*)f_data;
        Gait::Gait *g1 = data->gait1;
        Gait::Gait *g2 = data->gait2;
        long t1 = data->t1;
        long startTime = data->startTime;
        long duration = (long)(x[0]) - t1;
        std::chrono::nanoseconds startTime_ns(startTime);
        std::chrono::nanoseconds t1_ns(t1);
        std::chrono::nanoseconds t2_ns((long)x[1]);

        // x is vector {t_t, t_2}
        VectorXd P0 = g1->evaluate(t1_ns - startTime_ns);
        VectorXd P1 = P0 + ((double)duration/3)*g1->derivative(t1_ns - startTime_ns);
        VectorXd P3 = g2->evaluate(t2_ns);
        VectorXd P2 = P3 - ((double)duration/3)*g2->derivative(t2_ns);

        std::vector<VectorXd> points = {P0, P1, P2, P3};
        Bezier::Curve<long> curve(points, (long)(x[0]) - t1, t1);

        double avgVel = (1/(double)((long)x[0] - t1)) * (curve.dEvaluate(t1) - curve.dEvaluate((long)x[0])).norm();
        double duration_cost = (double)duration/1e9;

        avgVel = avgVel * 1e17;
        std::cout << "avgVel: " << avgVel << ", duration_cost: " << duration_cost << std::endl;

        return avgVel + duration_cost;
    }

    Bezier::Curve<long> findOptimalCubicCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        double t1_d = (double) t1.count();
        double period_d = (double) gait2->getPeriod().count();

        std::cout.precision(20);
        std::cout << "t1: " << t1.count() << ", period: " << gait2->getPeriod().count() << std::endl;
        std::cout << "t1_d: " << t1_d << ", period_d: " << period_d << std::endl;

        opt optimization = opt(LN_SBPLX, 2);
        optimization.set_lower_bounds({t1_d + 5e8, 0});
        //TODO robust way to set upper bound?
        optimization.set_upper_bounds({t1_d + 5e9, period_d});

        //add buffer to lower bound to prevent negative durations from casting
        std::cout << "setting lower bounds: {" << t1_d + 5e8 << ", " << 0 << "}" << std::endl;
        std::cout << "setting upper bounds: {" << t1_d + 5e9 << ", " << period_d << "}" << std::endl;

        optimization.set_maxtime(1);

        OptData data = {gait1, gait2, t1.count(), startTime.count()};
        optimization.set_min_objective(cost_func, &data);
        optimization.set_xtol_rel(1e-2);

        // guess is {t_t, t_2}
        std::vector<double> guess = {t1_d + 5e8 + period_d/2, period_d/2};
        double obj_value;

        try {
            result res = optimization.optimize(guess, obj_value);
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        } catch (std::exception &e) {
            std::printf("Optimization failed: %s \n", e.what());
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        }

        std::chrono::nanoseconds tt((long)guess[0]);
        std::chrono::nanoseconds t2((long)guess[1]);
        long duration = (long)(guess[0]) - t1.count();

        *t_t = tt;
        *t_2 = t2;

        // //generate bezier curve based on guess
        VectorXd P0 = gait1->evaluate(t1 - startTime);
        VectorXd P1 = P0 + ((double)duration/3)*gait1->derivative(t1 - startTime);
        VectorXd P3 = gait2->evaluate(t2);
        VectorXd P2 = P3 - ((double)duration/3)*gait2->derivative(t2);

        std::cout << "Bezier Curve: {" << P0(0) << ", " << P0(1) << "}, {" 
        << P1(0) << ", " << P1(1) << "}, {"
        << P2(0) << ", " << P2(1) << "}, {"
        << P3(0) << ", " << P3(1) << "}, "
        << "T: " << (long)(guess[0]) - t1.count() << ", t0: " << t1.count() << std::endl;
        return Bezier::Curve<long>({P0, P1, P2, P3}, duration, t1.count());
    }
}
