#ifndef SUPREME_CSL_CONTROL_HPP
#define SUPREME_CSL_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

inline double pos(double value) { return std::max(.0, value); }
inline double neg(double value) { return std::min(.0, value); }
inline double posneg(double value, double p, double n) { return p*pos(value) + n*neg(value); }

/**TODO:
 + make this class easy to use for all
 + remove update_mode from loop
 + make safe to use, careful init, bounds etc.
 */

class csl_control {
public:

    double z = 0.0;
    double target_csl_mode = .0;
    double target_csl_fb   = 1.03;
    double limit_lo        = -0.8;
    double limit_hi        = +0.8;

    double gi_pos =  4.0;
    double gi_neg = 16.0;

    double gi = .0;
    double gf = .0;

    const uint8_t id;
    const double dt;

    csl_control(uint8_t id = 0, double dt = .01/*100Hz*/) : id(id), dt(dt) { }

    void update_mode() {
        const double mode = clip(target_csl_mode);
        gi = posneg(mode, gi_pos, gi_neg);
        gf = target_csl_fb * pos(mode);
    }

    double step(double /*current position=*/p, double in = .0)
    {
        if (p > limit_hi) { z = std::min(z, gi * limit_hi); in = std::min(.0, in); }
        if (p < limit_lo) { z = std::max(z, gi * limit_lo); in = std::max(.0, in); }

        double u = clip(-gi * p + z + gi*in*dt, .5);
        update_mode(); /* change params here for z to have correct value */
        z = gi * p + gf * u;

        return u;
    }

    void reset(double p) {
        update_mode();
        z = gi * p; /* set initial conditions */
    }

};

} /* namespace supreme */

#endif /* SUPREME_CSL_CONTROL_HPP */

