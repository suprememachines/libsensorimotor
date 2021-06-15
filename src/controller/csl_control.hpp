#ifndef SUPREME_CSL_CONTROL_HPP
#define SUPREME_CSL_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

inline float pos(float value) { return std::max(.0f, value); }
inline float neg(float value) { return std::min(.0f, value); }
inline float posneg(float value, float p, float n) { return p*pos(value) + n*neg(value); }

/**TODO:
 + make this class easier to use
 + remove update_mode from loop
 + make safe to use, careful init, bounds etc.
 */

class csl_control {

    float z = 0.0;

public:

    float target_csl_mode = .0;
    float target_csl_fb   = 1.01;
    float limit_lo        = -0.8;
    float limit_hi        = +0.8;

    float gi_pos =  2.5;
    float gi_neg = 10.0;

    float gi = .0;
    float gf = .0;

    const uint8_t id;
    const float dt;

    const float eps = 0.01; /* limit margin to safely detect if Umax is reached */
    const float eta = 0.005; /* limit adaption rate */

    const float Umax;  /* limit of output voltage */

    float lim_hi = +Umax;
    float lim_lo = -Umax;

    csl_control(uint8_t id = 0, float dt = .01/*100Hz*/, float Umax = .5) : id(id), dt(dt), Umax(Umax) { }

    void update_mode() {
        const float mode = clip(target_csl_mode);
        gi = posneg(mode, gi_pos, gi_neg);
        gf = target_csl_fb * pos(mode);
    }

    float step(float /*current position=*/p, float in = .0)
    {
        float u = clip(-gi * p + z + gi*in*dt, lim_lo, lim_hi);

        if (u+eps >= lim_hi) lim_hi += (-0.01 - lim_hi) * eta;
        else                 lim_hi += ( Umax - lim_hi) * eta * 2;

        if (u-eps <= lim_lo) lim_lo += ( 0.01 - lim_lo) * eta;
        else                 lim_lo += (-Umax - lim_lo) * eta * 2;

        update_mode(); /* change params here for z to have correct value */

        z = gi * p + gf * u;

        //printf("%+5.2f %+5.2f %+5.2f\n",u, lim_lo, lim_hi);
        return u;
    }

    void reset(float p) {
        update_mode();
        z = gi * p; /* set initial conditions */
    }

};

} /* namespace supreme */

#endif /* SUPREME_CSL_CONTROL_HPP */

