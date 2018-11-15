#ifndef SUPREME_CSL_CONTROL_HPP
#define SUPREME_CSL_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

inline double pos(double value) { return std::max(.0, value); }
inline double neg(double value) { return std::min(.0, value); }
inline double posneg(double value, double p, double n) { return p*pos(value) + n*neg(value); }


class csl_control {
public:

    double z = 0.0;
    double target_csl_mode = .0;
    double target_csl_fb   = 1.03;
    double limit_lo        = -0.8;
    double limit_hi        = +0.8;

    double gi = .0;
    double gf = .0;

    uint8_t id;

    csl_control(uint8_t id) : id(id) { }

    /* TODO make value gi configurable */
    void update_mode() {
        const double mode = clip(target_csl_mode);
        gi = posneg(mode, 4.0, 16.0); /** TODO on gi change, reset z to correct value */
        gf = target_csl_fb * pos(mode);
    }

    double step(double /*current position=*/p)
    {
        update_mode();

        if (p > limit_hi) z = std::min(z, gi * p);
        if (p < limit_lo) z = std::max(z, gi * p);

        double u = clip(-gi * p + z, .5);
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

