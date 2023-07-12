#pragma once

#include <cmath>

namespace TrapezoidalTrajectory {

    template <typename X>
    class TrapTraj {

        public:
        TrapTraj() {
        }

        TrapTraj(double pmin, double pmax, double vmax, X t0, X tTransient) {
            p_orig_ = (vmax > 0 || pmin > pmax) ? pmin : pmax;
            pmin_ = pmin;
            pmax_ = pmax;
            vmax_ = (pmin < pmax) ? vmax : -std::abs(vmax);
            t0_ = t0;
            Tt_ = tTransient;
            Tc_ = std::abs((pmax - pmin) / vmax_) + Tt_;
            p_end_ = (p_orig_ == pmin) ? pmax : pmin;
            if (2*Tt_ > Tc_) {
                Tc_ = 2*Tt_;
                vmax_ = ((vmax > 0) ? 1 : -1) * (pmax - pmin) / (Tt_);
            }
            vmaxTt_ = vmax_/Tt_;
        }

        bool completed(X t) {
            return (t - t0_) > Tc_;
        }

        X duration() {
            return Tc_;
        }

        double velocity(X t) {
            if (this->completed(t)) {
                return 0;
            }

            t -= t0_;
            int sign = (t > Tc_/2) ? -1 : 1;

            if (t < Tt_) {
                return sign * vmaxTt_ * t;
            } else if (t < Tc_ - Tt_) {
                return sign * vmax_;
            } else {
                return sign * vmaxTt_ * (Tc_ - t);
            }

            return 0;
        }

        double position(X t) {
            if (this->completed(t)) {
                return p_end_;
            }

            t -= t0_;
            
            if (t < Tt_) {
                return 0.5 * vmaxTt_ * t * t + p_orig_;
            } else if (t < Tc_ - Tt_) {
                return vmax_ * (t - 0.5*Tt_) + p_orig_;
            } else {
                return vmax_ * (Tc_ - Tt_) - 0.5 * vmaxTt_ * (Tc_ - t) * (Tc_ - t) + p_orig_;
            }

            return 0;
        }

        X getT0() {
            return t0_;
        }

        private:
        double p_orig_;
        double p_end_;
        double pmin_;
        double pmax_;
        double vmax_;
        double vmaxTt_;
        X t0_;
        X Tt_;
        X Tc_;
    };
}