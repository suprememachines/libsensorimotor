#ifndef SUPREME_PID_CONTROL_HPP
#define SUPREME_PID_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

class pid_control {
public:

    double err_int = .0;
    double target_value = .0;

    double Kp = 0.8;
    double Ki = 0.0;

    uint8_t id;

    pid_control(uint8_t id) : id(id) {}

    double step(double current_value)
    {
        const double err = target_value - current_value;

        //dbg_msg("pos: %1.2f (%u)", current_value, id);
        err_int += err;
        err_int = clip(err_int);

        return Kp*err + Ki*err_int /**TODO Derivative part*/;
    }

    void reset(void) {
        err_int = 0.0;
    }
};

} /* namespace supreme */

#endif /* SUPREME_PID_CONTROL_HPP */
