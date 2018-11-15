/*
    SensorimotorPy
    Matthias Kubisch
    kubisch@informatik.hu-berlin.de
    October 2018

    This is a simple C-API for accessing the sensorimotors from python.

*/

#include "common/log_messages.h"
#include "motorcord.hpp"


class Motorhandler {
public:
    Motorhandler(unsigned number_of_motors, double /*update_rate_Hz*/, bool verbose)
    : motors(std::min(128u,number_of_motors), verbose)
    {
        sts_msg("Done starting motor cord.");
    }

    void execute_cycle()
    {
        motors.execute_cycle();
        /**TODO wait blocking correctly until next 10ms*/
        usleep(1000*8);
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
};

extern "C" {
    Motorhandler* sensorimotor_new(unsigned number_of_motors, double update_rate_Hz, bool verbose)
    {
        sts_msg("Starting motor cord.");
        return new Motorhandler(number_of_motors, update_rate_Hz, verbose);
    }

    int sensorimotor_del(Motorhandler* sensorimotor)
    {
        sts_msg("Stopping motor cord.");
        if (sensorimotor != NULL)
            delete sensorimotor;
        else
            wrn_msg("Motor cord already stopped (delete).");
        return true;
    }

    int sensorimotor_execute_cycle(Motorhandler* sensorimotor) {
        if (sensorimotor != NULL) {
            sensorimotor->execute_cycle();
            return 0;
        } else {
            wrn_msg("Motor cord already stopped (execute_cycle).");
            return -1;
        }
    }

    int sensorimotor_set_position(Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->set_position(data, N);
            return 0;
        } else {
            wrn_msg("Motor cord already stopped (set_position).");
            return -1;
        }
    }

    int sensorimotor_set_voltage_limit(Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->set_voltage_limit(data, N);
            return 0;
        } else {
            wrn_msg("Motor cord already stopped (set_voltage_limit).");
            return -1;
        }
    }

    int sensorimotor_apply_impulse(Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->apply_impulse(data, N);
            return 0;
        } else {
            wrn_msg("Motor cord already stopped (apply_impulse).");
            return -1;
        }
    }

    int sensorimotor_get_motor_data(Motorhandler* sensorimotor, double* data, unsigned N) {
        if (sensorimotor != NULL) {
            sensorimotor->get_motor_data(data, N);
            return 0;
        } else {
            wrn_msg("Motor cord already stopped (get_data).");
            return -1;
        }
    }

    int sensorimotor_ping(Motorhandler* sensorimotor) {
        if (sensorimotor == NULL) return -1;
        return sensorimotor->ping();
    }
}

