/*---------------------------------+
 | Supreme Machines                |
 | Sensorimotor C++ Library        |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | November 2018                   |
 +---------------------------------*/

#ifndef SUPREME_SENSORIMOTOR_HPP
#define SUPREME_SENSORIMOTOR_HPP

#include <cassert>
#include <algorithm>

#include "common/modules.h"
#include "communication_interface.hpp"
#include "interface_data.hpp"

#include "controller/pid_control.hpp"
#include "controller/csl_control.hpp"
#include "controller/impulse_ctrl.hpp"

/** TODO:

    + make the acceleration sensor mapping configurable
    + count timeouts and display
    + check communication reliability before driving motors
    + adjust timeout to actual max response time

*/


namespace supreme {

inline double uint16_to_sc(uint16_t word) { return (word - 32768) / 32768.0; }
inline double  int16_to_sc(uint16_t word) { return (int16_t) word / 32768.0; }

class sensorimotor
{
public:
    /* statistics */
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

private:

    static const unsigned max_response_time_us = 1000;
    static const unsigned byte_delay_us = 1;
    static const unsigned ping_timeout_us = 50;

    constexpr static const double voltage_scale = 0.012713472; /* Vmax = 13V -> 1023 */
    constexpr static const double current_scale = 0.003225806; /* Imax = 3A3 -> 1023 */

    const uint8_t             motor_id;
    communication_interface&  com;
    bool                      do_request = true;
    bool                      is_responding = false;
    bool                      voltage_limit_changed = false;
    bool                      read_external_sensor = false;

    int16_t                   direction = 1;
    double                    scalefactor = 1.0;
    double                    offset = 0.0;

    interface_data data;

    double                    target_voltage  = .0;
    double                    voltage_limit   = .0;
    double                    lim_disable_hi  = +0.90;
    double                    lim_disable_lo  = -0.90;

    pid_control     pos_ctrl;
    csl_control     csl_ctrl;
    impulse_control imp_ctrl;

    enum command_state_t {
        sync0,
        sync1,
        processing,
        completed,
        invalid,
    } syncstate = sync0;


    Statistics_t statistics;

public:

    enum Controller_t {
        none     = 0,
        voltage  = 1,
        position = 2,
        csl      = 3,
        impulse  = 4,
    } controller = none;

    sensorimotor(uint8_t id, communication_interface& com)
    : motor_id(id)
    , com(com)
    , data()
    , pos_ctrl(id)
    , csl_ctrl(id)
    , imp_ctrl(id)
    , statistics()
    {}

    /* returns the motors data, such as position, current etc. */
    const interface_data& get_data(void) const { return data; }

    /* returns the motors_id */
    uint8_t get_id(void) const { return motor_id; }

    /* returns the last known response to ping status */
    bool is_active(void) const { return is_responding; }

    /* disables the output stage of the motor by sending data requests only */
    void disable(void) { controller = Controller_t::none; }

    bool ping(void) {
        is_responding = false;
        enqueue_command_ping();
        com.read_msg();
        com.send_msg();
        receive_response(ping_timeout_us);
        return is_responding;
    }

    /* performs a full communication cycle */
    Statistics_t const& execute_cycle(void)
    {
        assert(send_motor_command()); /** TODO: handle connection lost better */
        receive_response();

        if (read_external_sensor) { /**TODO make sep. method*/
            enqueue_command_external_sensor_request();
            com.read_msg();
            com.send_msg();
            receive_response(/*ping_timeout_us*/);
        }

        return statistics;
    }

    const Statistics_t& get_stats(void) const { return statistics; }
    void reset_statistics(void) { statistics = Statistics_t(); }

    void set_controller_type(Controller_t type) { controller = type; }

    Controller_t get_controller_type(void) const { return controller; }

    void set_proportional(double p) { pos_ctrl.Kp = p; }
    void set_csl_limits(double lo, double hi) { csl_ctrl.limit_hi = hi; csl_ctrl.limit_lo = lo; }
    void set_target_csl_mode(double m) { csl_ctrl.target_csl_mode = m; }
    void set_target_csl_fb  (double f) { csl_ctrl.target_csl_fb   = f; }
    void set_target_position(double p) { pos_ctrl.target_value = p; }
    void set_target_voltage (double v) { target_voltage = clip(v, voltage_limit); }

    void set_voltage_limit(double limit) { voltage_limit = clip(limit, 0., 1.); voltage_limit_changed = true; }

    void apply_impulse(double value, unsigned duration) { imp_ctrl.value = value; imp_ctrl.duration = duration; }

    void set_direction(int16_t dir) { direction = dir; }
    void set_scalefactor(double scf) { scalefactor = scf; }
    void set_offset(double o) { offset = o; }

    void set_disable_position_limits(double lo, double hi) { lim_disable_lo = lo; lim_disable_hi = hi; }

    void set_ext_sensor_readout(bool enable) { read_external_sensor = enable; }

    void execute_controller(void)
    {

        if (not in_range(data.position, lim_disable_lo, lim_disable_hi))
            disable();

        if (controller == Controller_t::position) set_target_voltage( pos_ctrl.step(data.position) ); else pos_ctrl.reset();
        if (controller == Controller_t::csl     ) set_target_voltage( csl_ctrl.step(data.position) ); else csl_ctrl.reset(data.position);
        if (controller == Controller_t::impulse ) set_target_voltage( imp_ctrl.step()              ); else imp_ctrl.reset();
    }

private:

    /** TODO: enqueue sync bytes and checksum could be done by someone else since each package is affected */

    void enqueue_command_data_request() {
        com.enqueue_sync_bytes(0xFF);
        com.enqueue_byte(0xC0);
        com.enqueue_byte(motor_id);
        com.enqueue_checksum();
    }

    void enqueue_command_ping(void) {
        com.enqueue_sync_bytes(0xFF);
        com.enqueue_byte(0xE0);
        com.enqueue_byte(motor_id);
        com.enqueue_checksum();
    }

    void enqueue_command_set_voltage(double voltage) {
        data.output_voltage = voltage;
        voltage *= direction; // correct direction
        com.enqueue_sync_bytes(0xFF);
        if (voltage >= 0.0) {
            com.enqueue_byte(0xB0);
        } else {
            com.enqueue_byte(0xB1);
        }
        com.enqueue_byte(motor_id);
        uint8_t pwm = static_cast<uint8_t>(round(std::abs(voltage) * 255));
        com.enqueue_byte(pwm);
        com.enqueue_checksum();
    }

    void enqueue_command_set_voltage_limit(void) {
        if (not voltage_limit_changed) return;
        if (voltage_limit > 0.5)
            wrn_msg("Changing voltage limit to %1.2f for motor %u", voltage_limit, motor_id);
        com.enqueue_sync_bytes(0xFF);
        com.enqueue_byte(0xA0);
        com.enqueue_byte(motor_id);
        uint8_t lim_pwm = static_cast<uint8_t>(round(std::abs(voltage_limit) * 255));
        com.enqueue_byte(lim_pwm);
        com.enqueue_checksum();
        voltage_limit_changed = false;
    }

    void enqueue_command_external_sensor_request() {
        com.enqueue_sync_bytes(0xFF);
        com.enqueue_byte(0x40);
        com.enqueue_byte(motor_id);
        com.enqueue_byte(/*sensor_id=*/1);
        com.enqueue_checksum();
    }

    std::size_t send_motor_command(void) {
        enqueue_command_set_voltage_limit();
        if (controller != Controller_t::none)
            enqueue_command_set_voltage(target_voltage);
        else
            enqueue_command_data_request();
        com.read_msg(); // read all whats left
        return com.send_msg();
    }

    void receive_response(unsigned timeout_us = max_response_time_us)
    {
        /* wait for data until timeout */
        syncstate = sync0;
        unsigned t_us = 0;
        do {
            while(receive_data());
        } while(++t_us < timeout_us and is_pending() and com.wait_us(byte_delay_us));

        statistics.update(t_us, t_us >= timeout_us, !is_data_valid());
    }

    /* return code true means continue processing, false: wait for next byte */
    bool receive_data(void) {
        com.read_msg();

        switch(syncstate)
        {
            case sync0:
                if (com.empty()) return false;
                if (com.front() == 0xff) { /* receive and eat first sync byte */
                    syncstate = sync1;
                    com.reset_checksum();
                    com.get_byte();
                }
                else { /* Unexpected first sync byte */
                    com.pop(); /* remove byte and try again */
                }
                return true;

            case sync1:
                if (com.empty()) return false;
                if (com.front() == 0xff) { /* receive and eat second sync byte */
                    syncstate = processing;
                    com.get_byte();
                }
                else { /* Unexpected second sync byte */
                    com.pop();         // remove byte..
                    syncstate = sync0; // and try again
                }
                return true;

            case processing: {
                if (com.empty()) return false;
                uint8_t cmd = com.front();
                switch(cmd)
                {
                case 0x80: /* state data response */
                    if (com.size() > 12) { // cmd + id + 2pos + 2cur + 2uba + 2usu +2tmp + chk = 13
                        com.get_byte(); /* eat command byte */
                        uint8_t mid = com.get_byte();
                        if (mid == motor_id) {
                            data.position        = uint16_to_sc(com.get_word()) * direction * scalefactor + offset;
                            data.current         = com.get_word() * current_scale;
                            data.velocity        = int16_to_sc(com.get_word()) * direction * scalefactor;
                            data.voltage_supply  = com.get_word() * voltage_scale;
                            data.temperature     = static_cast<int16_t>(com.get_word()) / 100.0;
                            /**TODO implement voltage_backemf */
                            com.get_byte(); /* eat checksum */
                        }
                        syncstate = (motor_id == mid and com.is_checksum_ok()) ? completed : invalid;
                        is_responding = (syncstate == completed);
                        return true;
                    }
                    return false;

                case 0xE1: /* ping response */
                    if (com.size() > 2) {
                        com.get_byte(); /* eat command byte */
                        uint8_t mid = com.get_byte();
                        com.get_byte(); /* eat checksum */
                        syncstate = (motor_id == mid and com.is_checksum_ok()) ? completed : invalid;
                        is_responding = (syncstate == completed);
                        return true;
                    }
                    return false;

                case 0x41: /* external sensor response */
                    if (com.size() > 8) { // cmd + id + x(2), y(2), z(2) + chk = 9
                        com.get_byte(); /* eat command byte */
                        uint8_t mid = com.get_byte();
                        if (mid == motor_id) {
                            data.acceleration.x = ( static_cast<int16_t>(com.get_word()) -  4 ) / 2048.0; //TODO mapping!
                            data.acceleration.y = ( static_cast<int16_t>(com.get_word()) +  4 ) / 2048.0;
                            data.acceleration.z = ( static_cast<int16_t>(com.get_word()) - 35 ) / 2048.0;
                            com.get_byte(); /* eat checksum */
                        }

                        syncstate = (motor_id == mid and com.is_checksum_ok()) ? completed : invalid;
                        is_responding = (syncstate == completed);
                        return true;
                    }
                    return false;
                default:
                    /* received unknown command byte */
                    syncstate = invalid;
                    return false;
                } /* switch cmd */

                assert(false);
                return false;
                }

            case invalid:
                /* done, but failed */
                return false;

            case completed:
                /* done, message received correctly */
                return false;

            default:
                assert(false);
                return false;

        } /* switch syncstate */
        assert(false);
        return false;
    }



    bool is_pending(void) const { return syncstate != completed and syncstate != invalid; }
    bool is_data_valid(void) const { return syncstate != invalid; }

public:
    command_state_t get_syncstate(void) const { return syncstate; }

};


} /* namespace supreme */

#endif /* SUPREME_SENSORIMOTOR_HPP */
