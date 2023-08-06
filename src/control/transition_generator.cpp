#include "stm_control/control/transition_generator.hpp"

using namespace nlopt;
using namespace Eigen;

namespace TransitionGenerator {

    struct OptData {
        Gait::Gait *gait1;
        Gait::Gait *gait2;
        int64_t t1;
        int64_t startTime;
    };

    TransitionGait generate(TransitionType type, Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        switch (type) {
            case NAIVE:
                return TransitionGait(findNaiveTrajectory(gait1, gait2, startTime, t1, t_t, t_2));
            case LINEAR_WAYPOINT:
                return TransitionGait(findOptimalWaypointLinear(gait1, gait2, startTime, t1, t_t, t_2));
            case BEZIER_WAYPOINT:
                return TransitionGait(findOptimalWaypointCurve(gait1, gait2, startTime, t1, t_t, t_2));
            case BEZIER:
                Bezier::Curve<int64_t> curve = findOptimalCubicCurve(gait1, gait2, startTime, t1, t_t, t_2);
                return TransitionGait(curve);
        }
        return TransitionGait();
    }

    std::vector<TrapezoidalTrajectory::TrapTraj<int64_t>> findNaiveTrajectory(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        //TODO generalize to n joints
        VectorXd x0 = gait1->evaluate(t1 - startTime);
        // generate steering trajectory
        TrapezoidalTrajectory::TrapTraj<int64_t> steeringTrajectory = TrapezoidalTrajectory::TrapTraj<int64_t>(x0(0), -40*(M_PI/180.0), 15*(M_PI/180.0)/1e9, t1.count(), 2e8);
        // generate bogie trajectory
        TrapezoidalTrajectory::TrapTraj<int64_t> bogieTrajectory = TrapezoidalTrajectory::TrapTraj<int64_t>(x0(1), 0, 10*(M_PI/180.0)/1e9, t1.count(), 5e8);

        int64_t transitionDuration = std::max(steeringTrajectory.duration(), bogieTrajectory.duration());
        *t_t = std::chrono::nanoseconds(transitionDuration) + t1;
        *t_2 = std::chrono::nanoseconds(0);

        return {steeringTrajectory, bogieTrajectory};
    }

    double cost_func(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
        // cost calculations should be in units of seconds

        OptData *data = (OptData*)f_data;
        Gait::Gait *g1 = data->gait1;
        Gait::Gait *g2 = data->gait2;
        int64_t t1 = data->t1;
        int64_t startTime = data->startTime;
        int64_t duration = (int64_t)(x[0]) - t1;
        std::chrono::nanoseconds startTime_ns(startTime);
        std::chrono::nanoseconds t1_ns(t1);
        std::chrono::nanoseconds t2_ns((int64_t)x[1]);

        // x is vector {t_t, t_2}
        VectorXd P0 = g1->evaluate(t1_ns - startTime_ns);
        VectorXd P1 = P0 + ((double)duration/3)*g1->derivative(t1_ns - startTime_ns);
        VectorXd P3 = g2->evaluate(t2_ns);
        VectorXd P2 = P3 - ((double)duration/3)*g2->derivative(t2_ns);

        std::vector<VectorXd> points = {P0, P1, P2, P3};
        Bezier::Curve<int64_t> curve(points, (int64_t)(x[0]) - t1, t1);

        // add velocity cost for uniformly sampled points over curve
        double avgVel = 0;
        int n_steps = 50;
        int64_t time_delta = (int64_t)((x[0]) - t1)/(n_steps-1);
        for (int i = 0; i < n_steps; i++) {
            avgVel += curve.dEvaluate(i*time_delta).norm()/1e9;
        }

        // add duration cost for transition time
        double duration_cost = (double)duration/1e9;

        // calculate displacement caused by taking trajectory
        size_t num_trajectory_points = 100;
        std::vector<double> beta_list(num_trajectory_points);
        double step = x[1] / (num_trajectory_points - 1);
        for (size_t i = 0; i < num_trajectory_points; i++) {
            int64_t trajectory_time = t1 + step*i;
            beta_list[i] = curve.evaluate(trajectory_time)(0);
        }

        Vector3d gait1_coordinate = g1->displacement(t1_ns);
        Vector3d gait2_coordinate = g2->displacement(t2_ns);
        std::vector<double> trajectory_displacement = RoverTrajectory::calculate_rover_displacement(beta_list, step, {gait1_coordinate(0), gait1_coordinate(1), gait1_coordinate(2)}, P0(0)*(180.0/M_PI));
        Vector3d trajectory_displacement_vec; trajectory_displacement_vec << trajectory_displacement[0], trajectory_displacement[1], trajectory_displacement[2];

        Vector3d closest_point = RoverTrajectory::find_closest_waypoint(trajectory_displacement_vec, gait2_coordinate);
        double cartesian_trajectory_diff = (closest_point - trajectory_displacement_vec).norm();

        std::cout << "cartesian trajectory diff: " << cartesian_trajectory_diff << "avgVel: " << avgVel << ", duration_cost: " << duration_cost << std::endl;

        return avgVel + duration_cost + 20*cartesian_trajectory_diff;
    }

    Bezier::Curve<int64_t> findOptimalCubicCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        double t1_d = (double) t1.count();
        double period_d = (double) gait2->getPeriod().count();

        std::cout.precision(20);

        opt optimization = opt(LN_SBPLX, 2);
        optimization.set_lower_bounds({t1_d + 5e8, 0});
        //TODO robust way to set upper bound?
        optimization.set_upper_bounds({t1_d + 10e9, period_d});

        optimization.set_maxtime(0.5);

        OptData data = {gait1, gait2, t1.count(), startTime.count()};
        optimization.set_min_objective(cost_func, &data);
        optimization.set_xtol_rel(1e-2);

        // guess is {t_t, t_2}
        std::vector<double> guess = {t1_d + 7e8, period_d/2};
        double obj_value;

        try {
            result res = optimization.optimize(guess, obj_value);
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        } catch (std::exception &e) {
            std::printf("Optimization failed: %s \n", e.what());
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        }

        std::chrono::nanoseconds tt((int64_t)guess[0]);
        std::chrono::nanoseconds t2((int64_t)guess[1]);
        int64_t duration = (int64_t)(guess[0]) - t1.count();

        *t_t = tt;
        *t_2 = t2;

        // //generate bezier curve based on guess
        VectorXd P0 = gait1->evaluate(t1 - startTime);
        VectorXd P1 = P0 + ((double)duration/3)*gait1->derivative(t1 - startTime);
        VectorXd P3 = gait2->evaluate(t2);
        VectorXd P2 = P3 - ((double)duration/3)*gait2->derivative(t2);

        // std::cout << "Bezier Curve: {" << P0(0) << ", " << P0(1) << "}, {" 
        // << P1(0) << ", " << P1(1) << "}, {"
        // << P2(0) << ", " << P2(1) << "}, {"
        // << P3(0) << ", " << P3(1) << "}, "
        // << "T: " << (int64_t)(guess[0]) - t1.count() << ", t0: " << t1.count() << std::endl;
        return Bezier::Curve<int64_t>({P0, P1, P2, P3}, duration, t1.count());
    }

    ////////////////////////////////// Waypoint Optimization ////////////////////////////////////////////////////////////////////
    struct VelocityConstraintData {
        int i;
        VectorXd x0;
        int num_waypoints;
        double velocity_bound;
    };

    struct WaypointOptData {
        Gait::Gait *gait1;
        Gait::Gait *gait2;
        int64_t t1;
        int64_t startTime;
        int timesteps;
    };

    double steering_velocity_constraint(unsigned n, const double *x, double *grad, void *data) {
        VelocityConstraintData *d = (VelocityConstraintData*)data;
        int i = d->i;
        VectorXd x0 = d->x0;
        int num_waypoints = d->num_waypoints;
        double velocity_bound = d->velocity_bound;
        if (i == 0) {
            return std::abs((x[2] - x0(0))/(x[1]/num_waypoints)) - velocity_bound;
        } else {
            return std::abs((x[(i+1)*2] - x[2*i])/(x[1]/num_waypoints)) - velocity_bound;
        }
    }
    double bogie_velocity_constraint(unsigned n, const double *x, double *grad, void *data) {
        VelocityConstraintData *d = (VelocityConstraintData*)data;
        int i = d->i;
        VectorXd x0 = d->x0;
        int num_waypoints = d->num_waypoints;
        double velocity_bound = d->velocity_bound;
        if (i == 0) {
            return std::abs((x[3] - x0(1))/(x[1]/num_waypoints)) - velocity_bound;
        } else {
            return std::abs((x[2*(i+1)+1] - x[1+2*i])/(x[1]/num_waypoints)) - velocity_bound;
        }
    }

    double BezierWaypointCostFunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
        // x is [t_2, t_t, waypoints...]
        
        WaypointOptData *data = (WaypointOptData*)f_data;
        Gait::Gait *g1 = data->gait1;
        Gait::Gait *g2 = data->gait2;
        int64_t t1 = data->t1;
        int64_t startTime = data->startTime;
        int64_t duration = (int64_t)(x[1]);
        int n_timesteps = data->timesteps;
        std::chrono::nanoseconds startTime_ns(startTime);
        std::chrono::nanoseconds t1_ns(t1);
        std::chrono::nanoseconds t2_ns((int64_t)x[0]);
        
        double obj_value = 0;
        VectorXd x0(2); x0 = g1->evaluate(t1_ns - startTime_ns);
        VectorXd xf(2); xf = g2->evaluate(t2_ns);

        // minimize distance of points from boundary positions
        VectorXd final_waypoint(2); final_waypoint << x[2*n_timesteps], x[2*n_timesteps+1];
        double final_diff = (xf - final_waypoint).norm();

        // penalize points from bezier trajectory
        VectorXd P0 = x0;
        VectorXd P3 = xf;
        VectorXd P1 = P0 + g1->derivative(t1_ns - startTime_ns)*x[1]/3;
        VectorXd P2 = P3 - g2->derivative(t2_ns)*x[1]/3;
        Bezier::Curve<int64_t> curve({P0, P1, P2, P3}, duration, t1);
        int64_t time_delta = x[1]/n_timesteps;
        double time_delta_s = time_delta / 1e9;

        double bezier_traj_penalty = 0;
        for (int i = 1; i < n_timesteps+1; i++) {
            Eigen::VectorXd waypoint(2); waypoint << x[2*i], x[2*i+1];
            Eigen::VectorXd bezier_waypoint = curve.evaluate(t1 + time_delta*i);
            bezier_traj_penalty += (bezier_waypoint - waypoint).norm();
        }

        // penalize acceleration
        double acceleration_penalty = 0;
        for (int i = 1; i < n_timesteps; i++) {
            acceleration_penalty += std::abs(x[2*(i+1)] - x[2*i])/std::pow(time_delta_s, 2);

            acceleration_penalty += std::abs(x[2*(i+1)+1] - x[1+2*i])/std::pow(time_delta_s, 2);
        }

        // penalize time
        double time_penalty = x[1]/1e9;

        // calculate displacement caused by taking trajectory
        std::vector<VectorXd> waypoints(20+1, VectorXd(2));
        waypoints[0] = x0;
        for (int i = 1; i < 20+1; i++) {
            waypoints[i] << x[2*i], x[1+2*i];
        }
        Bezier::Spline<int64_t> spline = Bezier::Spline<int64_t>(waypoints, (int64_t)x[1], t1);

        size_t num_trajectory_points = 100;
        std::vector<double> beta_list(num_trajectory_points);
        double step = (double)x[1] / (double)(num_trajectory_points - 1);
        for (size_t i = 0; i < num_trajectory_points; i++) {
            int64_t trajectory_time = t1 + step*i;
            beta_list[i] = spline.evaluate(trajectory_time)(0);
        }

        Vector3d gait1_coordinate = g1->displacement(t1_ns);
        Vector3d gait2_coordinate = g2->displacement(t2_ns);
        std::vector<double> trajectory_displacement = RoverTrajectory::calculate_rover_displacement(beta_list, step, {gait1_coordinate(0), gait1_coordinate(1), gait1_coordinate(2)}, x0(0)*(180.0/M_PI));
        Vector3d trajectory_displacement_vec; trajectory_displacement_vec << trajectory_displacement[0], trajectory_displacement[1], trajectory_displacement[2];
        Vector3d closest_point = RoverTrajectory::find_closest_waypoint(trajectory_displacement_vec, gait2_coordinate);

        std::cout << "trajectory displacement: [" << trajectory_displacement[0] << ", " << trajectory_displacement[1] << ", " << trajectory_displacement[2] << "]" << std::endl;
        std::cout << "g2 displacement: [" << gait2_coordinate[0] << ", " << gait2_coordinate[1] << ", " << gait2_coordinate[2] << "]" << std::endl;
        std::cout << "closest_point: [" << closest_point[0] << ", " << closest_point[1] << ", " << closest_point[2] << "]" << std::endl;

        double cartesian_trajectory_diff = (closest_point - trajectory_displacement_vec).norm();

        std::cout << "final_diff: " << final_diff << " cartesian_trajectory_diff: " << cartesian_trajectory_diff << " bezier traj diff: " << bezier_traj_penalty << " acceleration penalty: " << acceleration_penalty << " time penalty: " << time_penalty << std::endl;
        
        obj_value = 200*final_diff + bezier_traj_penalty + 5*acceleration_penalty + 4*time_penalty + 20*cartesian_trajectory_diff;

        return obj_value;
    }

    double LinearWaypointCostFunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
        // x is [t_2, t_t, waypoints...]
        
        WaypointOptData *data = (WaypointOptData*)f_data;
        Gait::Gait *g1 = data->gait1;
        Gait::Gait *g2 = data->gait2;
        int64_t t1 = data->t1;
        int64_t startTime = data->startTime;
        int64_t duration = (int64_t)(x[1]);
        int n_timesteps = data->timesteps;
        std::chrono::nanoseconds startTime_ns(startTime);
        std::chrono::nanoseconds t1_ns(t1);
        std::chrono::nanoseconds t2_ns((int64_t)x[0]);
        
        double obj_value = 0;
        VectorXd x0(2); x0 = g1->evaluate(t1_ns - startTime_ns);
        VectorXd xf(2); xf = g2->evaluate(t2_ns);

        // minimize distance of points from boundary positions
        VectorXd final_waypoint(2); final_waypoint << x[2*n_timesteps], x[2*n_timesteps+1];
        double final_diff = (xf - final_waypoint).norm();

        // penalize points from linear trajectory
        double linear_traj_diff = 0;
        Eigen::VectorXd position_delta = (xf - x0)/n_timesteps;
        for (int i = 1; i < n_timesteps+1; i++) {
            Eigen::VectorXd waypoint(2); waypoint << x[2*i], x[2*i+1];
            Eigen::VectorXd linear_waypoint = x0 + position_delta*i;
            linear_traj_diff += (linear_waypoint - waypoint).norm();
        }

        // penalize acceleration
        double acceleration_penalty = 0;
        int64_t time_delta = x[1]/n_timesteps;
        double time_delta_s = time_delta / 1e9;
        for (int i = 1; i < n_timesteps; i++) {
            acceleration_penalty += std::abs(x[2*(i+1)] - x[2*i])/std::pow(time_delta_s, 2);

            acceleration_penalty += std::abs(x[2*(i+1)+1] - x[1+2*i])/std::pow(time_delta_s, 2);
        }

        // penalize time
        double time_penalty = x[1]/1e9;

        // calculate displacement caused by taking trajectory
        std::vector<VectorXd> waypoints(20+1, VectorXd(2));
        waypoints[0] = x0;
        for (int i = 1; i < 20+1; i++) {
            waypoints[i] << x[2*i], x[1+2*i];
        }
        Bezier::Spline<int64_t> spline = Bezier::Spline<int64_t>(waypoints, (int64_t)x[1], t1);

        size_t num_trajectory_points = 100;
        std::vector<double> beta_list(num_trajectory_points);
        double step = (double)x[1] / (double)(num_trajectory_points - 1);
        for (size_t i = 0; i < num_trajectory_points; i++) {
            int64_t trajectory_time = t1 + step*i;
            beta_list[i] = spline.evaluate(trajectory_time)(0);
        }

        Vector3d gait1_coordinate = g1->displacement(t1_ns);
        Vector3d gait2_coordinate = g2->displacement(t2_ns);
        std::vector<double> trajectory_displacement = RoverTrajectory::calculate_rover_displacement(beta_list, step, {gait1_coordinate(0), gait1_coordinate(1), gait1_coordinate(2)}, x0(0)*(180.0/M_PI));
        Vector3d trajectory_displacement_vec; trajectory_displacement_vec << trajectory_displacement[0], trajectory_displacement[1], trajectory_displacement[2];
        Vector3d closest_point = RoverTrajectory::find_closest_waypoint(trajectory_displacement_vec, gait2_coordinate);

        std::cout << "trajectory displacement: [" << trajectory_displacement[0] << ", " << trajectory_displacement[1] << ", " << trajectory_displacement[2] << "]" << std::endl;
        std::cout << "g2 displacement: [" << gait2_coordinate[0] << ", " << gait2_coordinate[1] << ", " << gait2_coordinate[2] << "]" << std::endl;
        std::cout << "closest_point: [" << closest_point[0] << ", " << closest_point[1] << ", " << closest_point[2] << "]" << std::endl;

        double cartesian_trajectory_diff = (closest_point - trajectory_displacement_vec).norm();

        std::cout << "final_diff: " << 200*final_diff << " cartesian_trajectory_diff: " << cartesian_trajectory_diff << " linear traj diff: " << linear_traj_diff << " acceleration penalty: " << 5*acceleration_penalty << " time penalty: " << 4*time_penalty << std::endl;
        
        obj_value = 200*final_diff + linear_traj_diff + 5*acceleration_penalty + 4*time_penalty + 20*cartesian_trajectory_diff;

        return obj_value;
    }

    Bezier::Spline<int64_t> findOptimalWaypointCurve(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        int num_waypoints = 20;
        double steering_velocity_bound = 15*(M_PI/180.0) / 1e9;
        double bogie_velocity_bound = 10*(M_PI/180.0) / 1e9;
        VectorXd x0 = gait1->evaluate(t1 - startTime);

        opt optimization = opt(LN_COBYLA, 2+num_waypoints*2);
        optimization.set_xtol_rel(1e-2);
        optimization.set_maxtime(0.5);

        // Add velocity constraints
        std::vector<VelocityConstraintData> velocityConstraints(num_waypoints*2);
        for (int i = 0; i < num_waypoints; i+=2) {
            VelocityConstraintData data = {i, x0, num_waypoints, steering_velocity_bound};
            velocityConstraints[i] = data;
            optimization.add_inequality_constraint(steering_velocity_constraint, &velocityConstraints[i], 1e-8);
            data = {i, x0, num_waypoints, bogie_velocity_bound};
            velocityConstraints[i+1] = data;
            optimization.add_inequality_constraint(bogie_velocity_constraint, &velocityConstraints[i], 1e-8);
        }

        // make initial guess
        double tt_guess = 2e9;
        int64_t gait_time = (t1.count() - startTime.count()) % gait1->getPeriod().count();
        double gait1_completion = (double)(gait_time)/(double)(gait1->getPeriod().count());
        int64_t t2_guess = gait1_completion * gait2->getPeriod().count();

        // std::printf("gait1 completion: %f, t2_guess: %f \n", gait1_completion, (double)t2_guess);

        std::vector<double> guess = {(double)t2_guess, tt_guess};
        VectorXd xf = gait2->evaluate(std::chrono::nanoseconds(t2_guess));

        VectorXd P0 = x0;
        VectorXd P3 = xf;
        VectorXd P1 = P0 + gait1->derivative(t1 - startTime)*tt_guess/3;
        VectorXd P2 = P3 - gait2->derivative(std::chrono::nanoseconds(0))*tt_guess/3;
        Bezier::Curve<int64_t> curve({P0, P1, P2, P3}, tt_guess, t1.count());

        int64_t guess_time_delta = tt_guess/num_waypoints;
        for (int i = 1; i < num_waypoints+1; i++) {
            VectorXd waypoint = curve.evaluate(t1.count() + guess_time_delta*i);
            guess.push_back(waypoint(0));
            guess.push_back(waypoint(1));
        }

        WaypointOptData data = {gait1, gait2, t1.count(), startTime.count(), num_waypoints};
        optimization.set_min_objective(BezierWaypointCostFunc, &data);

        double obj_value;
        try {
            result res = optimization.optimize(guess, obj_value);
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        } catch (std::exception &e) {
            std::printf("Optimization failed: %s \n", e.what());
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        }

        std::vector<VectorXd> waypoints(num_waypoints+1, VectorXd(2));
        waypoints[0] = x0;
        for (int i = 1; i < num_waypoints+1; i++) {
            waypoints[i] << guess[2*i], guess[1+2*i];
        }

        std::chrono::nanoseconds tt((int64_t)guess[1]);
        std::chrono::nanoseconds t2((int64_t)guess[0]);
        *t_t = tt + t1;
        *t_2 = t2;

        Bezier::Spline<int64_t> spline = Bezier::Spline<int64_t>(waypoints, guess[1], t1.count());
        return spline;
    }

    Bezier::Spline<int64_t> findOptimalWaypointLinear(Gait::Gait *gait1, Gait::Gait *gait2, std::chrono::nanoseconds startTime, std::chrono::nanoseconds t1, std::chrono::nanoseconds *t_t, std::chrono::nanoseconds *t_2) {
        int num_waypoints = 20;
        double steering_velocity_bound = 15*(180.0/M_PI) / 1e9;
        double bogie_velocity_bound = 10*(180.0/M_PI) / 1e9;
        VectorXd x0 = gait1->evaluate(t1 - startTime);

        opt optimization = opt(LN_COBYLA, 2+num_waypoints*2);
        optimization.set_xtol_rel(1e-2);
        optimization.set_maxtime(0.5);

        // Add velocity constraints
        std::vector<VelocityConstraintData> velocityConstraints(num_waypoints*2);
        for (int i = 0; i < num_waypoints; i+=2) {
            VelocityConstraintData data = {i, x0, num_waypoints, steering_velocity_bound};
            velocityConstraints[i] = data;
            optimization.add_inequality_constraint(steering_velocity_constraint, &velocityConstraints[i], 1e-8);
            data = {i, x0, num_waypoints, bogie_velocity_bound};
            velocityConstraints[i+1] = data;
            optimization.add_inequality_constraint(bogie_velocity_constraint, &velocityConstraints[i], 1e-8);
        }

        // make initial guess
        double tt_guess = 2e9;
        int64_t gait_time = (t1.count() - startTime.count()) % gait1->getPeriod().count();
        double gait1_completion = (double)(gait_time)/(double)(gait1->getPeriod().count());
        int64_t t2_guess = gait1_completion * gait2->getPeriod().count();

        // std::printf("gait1 completion: %f, t2_guess: %f \n", gait1_completion, (double)t2_guess);

        std::vector<double> guess = {(double)t2_guess, tt_guess};
        VectorXd xf = gait2->evaluate(std::chrono::nanoseconds(t2_guess));
        VectorXd linear_step = (xf - x0)/num_waypoints;
        for (int i = 1; i < num_waypoints+1; i++) {
            VectorXd waypoint = x0 + linear_step*i;
            guess.push_back(waypoint(0));
            guess.push_back(waypoint(1));
        }

        WaypointOptData data = {gait1, gait2, t1.count(), startTime.count(), num_waypoints};
        optimization.set_min_objective(LinearWaypointCostFunc, &data);

        double obj_value;
        try {
            result res = optimization.optimize(guess, obj_value);
            std::printf("Optimized Vector {%f, %f} w/ t1: {%li}, with objective value %f \n", guess[0], guess[1], t1.count(), obj_value);
        } catch (std::exception &e) {
            std::printf("Optimization failed: %s \n", e.what());
            std::printf("Optimized Vector {%f, %f}, with objective value %f \n", guess[0], guess[1], obj_value);
        }

        std::vector<VectorXd> waypoints(num_waypoints+1, VectorXd(2));
        waypoints[0] = x0;
        for (int i = 1; i < num_waypoints+1; i++) {
            waypoints[i] << guess[2*i], guess[1+2*i];
        }

        std::chrono::nanoseconds tt((int64_t)guess[1]);
        std::chrono::nanoseconds t2((int64_t)guess[0]);
        *t_t = tt + t1;
        *t_2 = t2;

        Bezier::Spline<int64_t> spline = Bezier::Spline<int64_t>(waypoints, guess[1], t1.count());
        return spline;
    }
}
