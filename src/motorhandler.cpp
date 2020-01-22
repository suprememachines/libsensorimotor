/*---------------------------------+
 | Supreme Machines                |
 | Sensorimotor C++ Library        |
 | Matthias Kubisch                |
 | Jetpack Cognition Lab           |
 | kubisch@informatik.hu-berlin.de |
 | January 2020                    |
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
    Motorhandler(uint8_t number_of_motors, float update_rate_Hz, bool verbose)
    : motors(std::min((uint8_t)128, number_of_motors), update_rate_Hz, verbose)
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

    void set_position(float* data, uint8_t N)
    {
        assertion(motors.size() == N, "Cannot set position. Array size does not match.");
        for (uint8_t i = 0; i < N; ++i)
        {
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::position);
            motors[i].set_target_position(data[i]);
        }
    }

    void set_pos_ctrl_params(uint8_t id, float* par, uint8_t N)
    {
        assertion(id < motors.size(), "Invalid motor ID: %u (expected 0-%u).", id, motors.size()-1);
        assertion(N == 5, "Invalid number of control parameters: %u (expected %u).", N, 5);
        motors[id].set_pos_ctrl_params(par[0], par[1], par[2], par[3], par[4]);
    }

    void set_voltage_limit(float* lim, uint8_t N)
    {
        assertion(motors.size() == N, "Cannot set voltage limit. Array size does not match.");
        for (uint8_t i = 0; i < N; ++i)
            motors[i].set_voltage_limit(lim[i]);
    }

    void apply_impulse(float* val, float* dur, uint8_t N)
    {
        assert(motors.size() == N);
        for (uint8_t i = 0; i < N; ++i)
        {
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::impulse);
            motors[i].set_disable_position_limits(-1.0,+1.0);
            motors[i].apply_impulse(val[i], dur[i]);
        }
    }

    /**TODO: use a struct? */
    void get_motor_data( float* pos
                       , float* vel
                       , float* cur
                       , float* vol
                       , float* tmp
                       , uint8_t N)
    {
        assert(motors.size() == N);
        for (uint8_t i = 0; i < N; ++i) {
            auto const& m = motors[i].get_data();
            pos[i] = m.position;
            vel[i] = m.velocity;
            cur[i] = m.current;
            vol[i] = m.voltage_supply;
            tmp[i] = m.temperature;
        }
    }


    uint8_t ping() {
        return motors.scan_for_motors();
    }


    //void set_raw_data(uint8_t id, uint8_t* data, uint8_t N) { /*TODO float or byte data? */}

    //void get_raw_data(uint8_t id, uint8_t* data, uint8_t N) {}

private:
    supreme::motorcord motors;
    supreme::SimpleTimer timer;
};

} /* namespace supreme */

inline void warning(const char* where) { wrn_msg("Assertion in '%s'", where); }

#define SAFE_EXEC(OBJ, FUNC) \
{                            \
   if (OBJ != nullptr) {     \
    FUNC;                    \
    return 0;                \
  } else {                   \
      warning(#FUNC);        \
      return -1;             \
  }                          \
}                            \



extern "C" {
    supreme::Motorhandler* sensorimotor_new(uint8_t number_of_motors, float update_rate_Hz, bool verbose)
    {
        sts_msg("Starting motor cord.");
        return new supreme::Motorhandler(number_of_motors, update_rate_Hz, verbose);
    }

    int sensorimotor_del(supreme::Motorhandler* sensorimotor)
    {
        sts_msg("Stopping motor cord.");
        SAFE_EXEC(sensorimotor, delete sensorimotor)
    }

    int sensorimotor_execute_cycle(supreme::Motorhandler* sensorimotor) {
        SAFE_EXEC(sensorimotor, sensorimotor->execute_cycle())
    }

    int sensorimotor_set_position(supreme::Motorhandler* sensorimotor, float* pos, uint8_t N) {
        SAFE_EXEC(sensorimotor, sensorimotor->set_position(pos, N))
    }

    int sensorimotor_set_pos_ctrl_params(supreme::Motorhandler* sensorimotor, uint8_t id, float* par, uint8_t N) {
        SAFE_EXEC(sensorimotor, sensorimotor->set_pos_ctrl_params(id, par, N))
    }

    int sensorimotor_set_voltage_limit(supreme::Motorhandler* sensorimotor, float* lim, uint8_t N) {
        SAFE_EXEC(sensorimotor, sensorimotor->set_voltage_limit(lim, N))
    }

    int sensorimotor_apply_impulse(supreme::Motorhandler* sensorimotor, float* val, float* dur, uint8_t N) {
        SAFE_EXEC(sensorimotor, sensorimotor->apply_impulse(val, dur, N))
    }

    int sensorimotor_get_motor_data( supreme::Motorhandler* sensorimotor
                                   , float* pos
                                   , float* vel
                                   , float* cur
                                   , float* vol
                                   , float* tmp
                                   , uint8_t N) {
        SAFE_EXEC(sensorimotor, sensorimotor->get_motor_data(pos, vel, cur, vol, tmp, N))
    }

    int sensorimotor_ping(supreme::Motorhandler* sensorimotor) {
        if (sensorimotor == nullptr) return -1;
        return sensorimotor->ping();
    }
}

