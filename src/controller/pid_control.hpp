#ifndef SUPREME_PID_CONTROL_HPP
#define SUPREME_PID_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

    /** Low-frequency pulse anti-friction mode:
        If the output is too small to actually move the motor, use irregular pulses of duration dt and
        a minimum height of 'threshold'. These pulses make the motors overcome static friction.
        The probability is proportional to the duty cycle 'value/threshold',
        and is zero below a certain dead_band.
     */

    float chop(float value, float threshold, float deadband) {
        if (std::abs(value) < threshold)
        {
            const float prob = std::abs(value)/threshold;
            return (random_value(deadband, 1.f) < prob) ? threshold*sign(value) : 0.f;
        }
        else return in_range(value, -deadband, deadband) ? 0.f : value;
    }


class pid_control {

    uint8_t id;
    const float dt;

    float err_int = 0.f;
    float err_lst = 0.f;
    float target_value = 0.f;

    float Kp = 0.f;
    float Ki = 0.f;
    float Kd = 0.f;

    float limit = 1.f;
    float pulse_mode_threshold = 0.f;
    float dead_band = 0.f;

public:

    pid_control(uint8_t id, float timestep) : id(id), dt(timestep) { dbg_msg("creating PID controller with time step: %1.4f", timestep); }

    float step(float current_value)
    {
        const float err = target_value - current_value;
        const float err_dif = err - err_lst;

        //dbg_msg("pos: %1.2f (%u)", current_value, id);
        /**TODO integral anti-windup */
        err_int += err;
        err_int = clip(err_int);

        err_lst = err;

        const float out = Kp*err + Ki*err_int*dt + Kd*err_dif/dt;

        return chop(clip(out, limit), pulse_mode_threshold, dead_band);
    }

    void set_pid(float p, float i, float d) {
        assert(p >= 0.f); Kp = p;
        assert(i >= 0.f); Ki = i;
        assert(d >= 0.f); Kd = d;
    }

    void set_target_value        (float t) {                   target_value         = t; }
    void set_limit               (float l) { assert(l >= 0.f); limit                = l; }
    void set_dead_band           (float d) { assert(d >= 0.f); dead_band            = d; }
    void set_pulse_mode_threshold(float t) { assert(t >= 0.f); pulse_mode_threshold = t; }

    void reset(void) {
        err_int = 0.f;
        err_lst = 0.f;
    }
};

} /* namespace supreme */

#endif /* SUPREME_PID_CONTROL_HPP */
