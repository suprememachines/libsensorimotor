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

    struct interface_data {
        /**TODO include motor_id*/
        uint8_t id = 0;

        float output_voltage  = 0.0;
        float position        = 0.0;
        //TODO remove:
            float last_p = 0.0; //TODO rename to last_pos
        float velocity        = 0.0;
        float current         = 0.0;
        float voltage_backemf = 0.0;
        float voltage_supply  = 0.0;
        float temperature     = 0.0;
        struct Acceleration { float x,y,z; } acceleration = {0.,0.,0.};
        Statistics_t statistics = {};

        uint8_t raw_send[256] = {};
        uint8_t raw_recv[256] = {};

        uint8_t num_sendbytes = 0;
    };

}

#endif /* SUPREME_INTERFACE_DATA_HPP */
