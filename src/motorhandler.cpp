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
        check_size(N);
        for (uint8_t i = 0; i < N; ++i)
        {
            motors[i].set_controller_type(sensorimotor::Controller_t::position);
            motors[i].set_target_position(data[i]);
        }
    }

    void set_position_id(float pos, uint8_t id)
    {
        check_id(id);
        motors[id].set_controller_type(sensorimotor::Controller_t::position);
        motors[id].set_target_position(pos);
    }

    void set_pos_ctrl_params(uint8_t id, float* par, uint8_t N)
    {
        check_id(id);
        assertion(N == 5, "Invalid number of control parameters: %u (expected %u).", N, 5);
        motors[id].set_pos_ctrl_params(par[0], par[1], par[2], par[3], par[4]);
    }

    void set_voltage_limit(float* lim, uint8_t N)
    {
        check_size(N);
        for (uint8_t i = 0; i < N; ++i)
            motors[i].set_voltage_limit(lim[i]);
    }

    void apply_impulse(float* val, float* dur, uint8_t N)
    {
        check_size(N);
        for (uint8_t i = 0; i < N; ++i)
        {
            motors[i].set_controller_type(sensorimotor::Controller_t::impulse);
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
        check_size(N);
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


    /**TODO consider using float instead of byte data. */

    void set_raw_data_send(uint8_t id, uint8_t* data, uint8_t N) {
        check_id(id);
        auto& m = motors[id].set_data();
        for (uint8_t i = 0; i < N; ++i)
            m.raw_send[i] = data[i];
        m.num_sendbytes = N;
        motors[id].set_controller_type(sensorimotor::Controller_t::send_raw);
    }

    void get_raw_data_recv(uint8_t id, uint8_t* data, uint8_t N) const {
        check_id(id);
        auto const& m = motors[id].get_data();
        for (uint8_t i = 0; i < N; ++i)
            data[i] = m.raw_recv[i];
    }


private:

    void check_id  (uint8_t id) const { assertion(id < motors.size(), "Invalid motor ID: %u (expected 0-%u).", id, motors.size()-1); }
    void check_size(uint8_t  N) const { assertion(motors.size() == N, "Array size does not match."); }

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

    int sensorimotor_del(supreme::Motorhandler* ux)
    {
        sts_msg("Stopping motor cord.");
        SAFE_EXEC(ux, delete ux)
    }

    int sensorimotor_execute_cycle(supreme::Motorhandler* ux) {
        SAFE_EXEC(ux, ux->execute_cycle())
    }

    int sensorimotor_set_position(supreme::Motorhandler* ux, float* pos, uint8_t N) {
        SAFE_EXEC(ux, ux->set_position(pos, N))
    }

    int sensorimotor_set_position_id(supreme::Motorhandler* ux, float pos, uint8_t id) {
        SAFE_EXEC(ux, ux->set_position_id(pos, id))
    }

    int sensorimotor_set_pos_ctrl_params(supreme::Motorhandler* ux, uint8_t id, float* par, uint8_t N) {
        SAFE_EXEC(ux, ux->set_pos_ctrl_params(id, par, N))
    }

    int sensorimotor_set_voltage_limit(supreme::Motorhandler* ux, float* lim, uint8_t N) {
        SAFE_EXEC(ux, ux->set_voltage_limit(lim, N))
    }

    int sensorimotor_apply_impulse(supreme::Motorhandler* ux, float* val, float* dur, uint8_t N) {
        SAFE_EXEC(ux, ux->apply_impulse(val, dur, N))
    }

    int sensorimotor_get_motor_data( supreme::Motorhandler* ux
                                   , float* pos
                                   , float* vel
                                   , float* cur
                                   , float* vol
                                   , float* tmp
                                   , uint8_t N) {
        SAFE_EXEC(ux, ux->get_motor_data(pos, vel, cur, vol, tmp, N))
    }

    int sensorimotor_set_raw_data_send(supreme::Motorhandler* ux, uint8_t id, uint8_t* data, uint8_t N) {
        SAFE_EXEC(ux, ux->set_raw_data_send(id, data, N));
    }

    int sensorimotor_get_raw_data_recv(supreme::Motorhandler* ux, uint8_t id, uint8_t* data, uint8_t N) {
        SAFE_EXEC(ux, ux->get_raw_data_recv(id, data, N));
    }

    int sensorimotor_ping(supreme::Motorhandler* ux) {
        if (ux == nullptr) return -1;
        return ux->ping();
    }
}

