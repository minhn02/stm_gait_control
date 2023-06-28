#include "stm_control/control/wheel_controller.hpp"

namespace WheelController {
    std::vector<double> calculateWheelSpeeds(double steeringAngle, double steeringVelocity) {
        std::vector<double> wheelSpeeds(4);
        std::vector<double> vSteering(4);
        std::vector<double> cDifferential(4);
        std::vector<double> vRover(4);
        
        double b_2 = steeringAngle * 0.5;
        double db_2dt = steeringVelocity * 0.5;

        // calculate the wheel velocities induced by the steering velocity
        for (int i = 0; i < 4; i++) {
            vSteering[i] = ((i/2) ? -1 : 1)*(-LX*std::tan(b_2) + ((i%2) ? -1 : 1)*LY)*db_2dt;
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
        std::vector<double> maxRoverSpeedPerWheel(4);
        for (int i = 0; i < 4; i++) {
            maxRoverSpeedPerWheel[i] = (MAX_WHEEL_SPEED*WHEEL_RADIUS - vSteering[i])/cDifferential[i];
        }
        
        double roverSpeedCapped = std::min(maxVRover, *std::min_element(maxRoverSpeedPerWheel.begin(), maxRoverSpeedPerWheel.end()));
        
        // calculate the final wheel speeds
        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] = (roverSpeedCapped * cDifferential[i] + vSteering[i])/WHEEL_RADIUS;
        }

        return wheelSpeeds;
    }
}