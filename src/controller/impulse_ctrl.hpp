#ifndef SUPREME_IMPULSE_CONTROL_HPP
#define SUPREME_IMPULSE_CONTROL_HPP

#include "../common/modules.h"
#include "../common/log_messages.h"

namespace supreme {

class impulse_control {
public:

    unsigned duration = 0;
    double value = 0.0;

    uint8_t id;

    impulse_control(uint8_t id) : id(id) {}

    double step()
    {
        if (duration > 0) {
            --duration;
            return value;
        } else return .0;
    }

    void reset(void) {
        duration = 0;
        value = .0;
    }
};

} /* namespace supreme */

#endif /* SUPREME_IMPULSE_CONTROL_HPP */

