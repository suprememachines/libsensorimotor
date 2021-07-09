#ifndef SUPREME_INTERFACE_DATA_HPP
#define SUPREME_INTERFACE_DATA_HPP

namespace supreme {

    struct Statistics_t {
        unsigned errors = 0;
        unsigned timeouts = 0;
        unsigned response_time_us = 0;
        float    avg_resp_time_us = 0.0;
        unsigned max_resp_time_us = 0;

        bool faulted = false;

        void update(unsigned time_us, bool timeout, bool invalid) {
            if (invalid) ++errors;
            if (timeout) ++timeouts;
            faulted = timeout or invalid;
            response_time_us = time_us;
            avg_resp_time_us = 0.99*avg_resp_time_us + 0.01*time_us;
            max_resp_time_us = std::max(max_resp_time_us, time_us);
        }

    };

    struct interface_data
    {
        uint8_t id = 0;

        float output_voltage  = .0f;
        float position        = .0f;
        float position_0      = .0f;
        float velocity        = .0f;
        float velocity_lpf    = .0f;
        float current         = .0f;
        float voltage_supply  = .0f;
        float temperature     = .0f;
        struct Acceleration { float x,y,z; } acceleration = {.0f,.0f,.0f};
        Statistics_t statistics = {};

        uint8_t raw_send[256] = {};
        uint8_t raw_recv[256] = {};

        uint8_t num_sendbytes = 0;
    };

}

#endif /* SUPREME_INTERFACE_DATA_HPP */
