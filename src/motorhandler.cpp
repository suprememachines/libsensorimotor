/*---------------------------------+
 | Supreme Machines                |
 | Sensorimotor C++ Library        |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | November 2018                   |
 +---------------------------------*/

/* This is a simple C-API for accessing the sensorimotors from python.
*/

#include "common/log_messages.h"
#include "common/timer.h"
#include "motorcord.hpp"

namespace supreme {

namespace constants {
    const uint64_t us_per_sec = 1000*1000;
}

class Motorhandler {
public:
    Motorhandler(unsigned number_of_motors, double update_rate_Hz, bool verbose)
    : motors(std::min(128u,number_of_motors), update_rate_Hz, verbose)
    , timer( static_cast<uint64_t>(constants::us_per_sec/update_rate_Hz), /*enable=*/true )
    {
        sts_msg("Done starting motor cord at %.2f Hz.", update_rate_Hz);
        /**TODO perform a communication test before sending pwm values to the motors. */
    }

    void execute_cycle()
    {
        motors.execute_cycle();

        while(!timer.check_if_timed_out_and_restart())
            usleep(100);
    }

    void set_position(double* data, unsigned N)
    {
        unsigned M = std::min((unsigned)motors.size(), N);
        for (unsigned i = 0; i < M; ++i)
        {
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::position);
            motors[i].set_target_position(data[i]);
        }
    }

    void set_pos_ctrl_params(unsigned motor_id, double* data, unsigned N)
    {
        if (N >= 5 and motor_id < motors.size()) {
            motors[motor_id].set_pos_ctrl_params(data[0], data[1], data[2], data[3], data[4]);
        }
        else wrn_msg("Too few parameters or wrong motor id");
    }

    void set_voltage_limit(double* data, unsigned N)
    {
        unsigned M = std::min((unsigned)motors.size(), N);
        for (unsigned i = 0; i < M; ++i)
            motors[i].set_voltage_limit(data[i]);
    }

    void apply_impulse(double* data, unsigned N)
    {
        unsigned M = std::min((unsigned)motors.size(), N);
        for (unsigned i = 0; i < M; ++i)
        {
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::impulse);
            motors[i].set_disable_position_limits(-1.0,+1.0);
            motors[i].apply_impulse(data[i], 5/**TODO duration*/);
        }
    }

    void get_motor_data(double* data, unsigned N)
    {
        unsigned M = std::min((unsigned)motors.size(), N);
        for (unsigned i = 0; i < M; ++i)
        {
            data[i] = motors[i].get_data().position; /**TODO other data as well*/
        }
    }

    uint8_t ping() {
        return motors.scan_for_motors();
    }

    ~Motorhandler() { sts_msg("Done stopping motor cord."); }

private:
    supreme::motorcord motors;
    supreme::SimpleTimer timer;
};

} /* namespace supreme */

inline void warning(const char* where) { wrn_msg("Motor cord already stopped in '%s'", where); }


extern "C" {
    supreme::Motorhandler* sensorimotor_new(unsigned number_of_motors, double update_rate_Hz, bool verbose)
    {
        sts_msg("Starting motor cord.");
        return new supreme::Motorhandler(number_of_motors, update_rate_Hz, verbose);
    }

    int sensorimotor_del(supreme::Motorhandler* sensorimotor)
    {
        sts_msg("Stopping motor cord.");
        if (sensorimotor != NULL)
            delete sensorimotor;
        else
            warning("delete");
        return true;
    }

    int sensorimotor_execute_cycle(supreme::Motorhandler* sensorimotor) {
        if (sensorimotor != NULL) {
            sensorimotor->execute_cycle();
            return 0;
        } else {
            warning("execute_cycle");
            return -1;
        }
    }

    int sensorimotor_set_position(supreme::Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->set_position(data, N);
            return 0;
        } else {
            warning("set_position");
            return -1;
        }
    }

    int sensorimotor_set_pos_ctrl_params(supreme::Motorhandler* sensorimotor, unsigned mid, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->set_pos_ctrl_params(mid, data, N);
            return 0;
        } else {
            warning("set_pos_ctrl_params");
            return -1;
        }


    }

    int sensorimotor_set_voltage_limit(supreme::Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->set_voltage_limit(data, N);
            return 0;
        } else {
            warning("set_voltage_limit");
            return -1;
        }
    }

    int sensorimotor_apply_impulse(supreme::Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->apply_impulse(data, N);
            return 0;
        } else {
            warning("apply_impulse");
            return -1;
        }
    }

    int sensorimotor_get_motor_data(supreme::Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->get_motor_data(data, N);
            return 0;
        } else {
            warning("get_data");
            return -1;
        }
    }

    int sensorimotor_ping(supreme::Motorhandler* sensorimotor) {
        if (sensorimotor == NULL) return -1;
        return sensorimotor->ping();
    }
}

