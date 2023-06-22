#include "stm_control/control/wheel_controller.hpp"

namespace WheelController {
    std::vector<double> calculateWheelSpeeds(double velocity, double steeringAngle, double steeringVelocity) {
        std::vector<double> wheelSpeeds(4);
        std::vector<double> vSteering(4);
        std::vector<double> cDifferential(4);
        std::vector<double> vRover(4);

        // calculate the wheel velocities induced by the steering velocity
        for (int i = 1; i < 5; i++) {
            vSteering[i-1] = steeringVelocity/2.0 * (std::pow(-1, i*(i+1)/2.0)*LX*std::tan(steeringAngle/2.0) + std::pow(-1, i*(i+1)/2.0)*LY);
        }

        // calculate the coefficient to apply to each wheel to achieve a certain rover velocity
        for (int i = 1; i < 5; i++) {
            cDifferential[i-1] = 1 - std::pow(-1, i)*(LX/LY)*std::tan(steeringAngle/2.0);
        }

        // calculate the velocity of the rover
        double maxVRover = 0;
        for (int i = 0; i < 4; i++) {
            double vel = -vSteering[i]/cDifferential[i];
            if (vel > maxVRover) {
                maxVRover = vel;
            }
        }

        maxVRover += velocity;
        // calculate the final wheel speeds
        for (int i = 0; i < 4; i++) {
            
            //TODO added negative sign to fix direction of rover
            wheelSpeeds[i] = -(maxVRover * cDifferential[i] + vSteering[i])/WHEEL_RADIUS;
        }

        return wheelSpeeds;
    }
}