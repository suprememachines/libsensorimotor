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

inline float uint16_to_sc(uint16_t word) { return (word - 32768) / 32768.f; }
inline float  int16_to_sc(uint16_t word) { return (int16_t) word / 32768.f; }

class sensorimotor
{
    static const unsigned max_response_time_us = 250;
    static const unsigned byte_delay_us = 10;
    static const unsigned ping_timeout_us = 500;

    constexpr static const float unit_V = 0.001f;  /* 1.0 mV */
    constexpr static const float unit_A = 0.0001f; /* 0.1 mA */
    constexpr static const float unit_C = 0.01f;   /* 10. mC */
    constexpr static const float vel_scale = 4.0;

    const uint8_t             motor_id;
    communication_interface&  com;
    bool                      do_request = true;
    bool                      is_responding = false;
    bool                      voltage_limit_changed = false;
    bool                      read_external_sensor = false;

    int16_t                   direction = 1;
    float                     scalefactor = 1.f;
    float                     offset = 0.f;
    float                     inv_dt;

    interface_data            data;

    float                     target_voltage  = .0f;
    float                     voltage_limit   = .0f;
    float                     lim_disable_hi  = +0.90f;
    float                     lim_disable_lo  = -0.90f;

    uint16_t                  target_period_us = 0;

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

public:

    enum Controller_t : uint8_t {
        none     = 0,
        voltage  = 1,
        position = 2,
        csl      = 3,
        impulse  = 4,
        send_raw = 5,
    } controller = none
    , targetctrl = none;

    sensorimotor(uint8_t id, communication_interface& com, float update_rate_Hz)
    : motor_id(id)
    , com(com)
    , inv_dt(update_rate_Hz/vel_scale)
    , data()
    , pos_ctrl(id, 1.f/update_rate_Hz)
    , csl_ctrl(id)
    , imp_ctrl(id)
    {
        dbg_msg("Velocity factor: %1.2f", inv_dt);
    }

    /* returns the motors data, such as position, current etc. */
    const interface_data& get_data(void) const { return data; }
          interface_data& set_data(void)       { return data; }

    /* returns the motors_id */
    uint8_t get_id(void) const { return motor_id; }

    /* returns the last known response to ping status */
    bool is_active(void) const { return is_responding; }

    /* disables the output stage of the motor by sending data requests only */
    void disable(void) { targetctrl = Controller_t::none; set_target_voltage(0.); }

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

        return data.statistics;
    }

    const Statistics_t& get_stats(void) const { return data.statistics; }
    void reset_statistics(void) { data.statistics = Statistics_t(); }

    void set_controller_type(Controller_t c) { targetctrl = c; }

    Controller_t get_controller_type(void) const { return controller; }

    void set_pos_ctrl_params(float Kp, float Ki, float Kd, float db = 0.f, float pt = 0.f) {
        pos_ctrl.set_pid(Kp, Ki, Kd);
        pos_ctrl.set_dead_band(db);
        pos_ctrl.set_pulse_mode_threshold(pt);
    }

    void set_csl_limits(float lo, float hi) { csl_ctrl.limit_hi = hi; csl_ctrl.limit_lo = lo; }
    void set_target_csl_mode(float m) { csl_ctrl.target_csl_mode = m; }
    void set_target_csl_fb  (float f) { csl_ctrl.target_csl_fb   = f; }
    void set_target_csl_gain(float g) { csl_ctrl.gi_pos   = g; }
    void set_target_position(float p) { pos_ctrl.set_target_value(p); }
    void set_target_voltage (float v) { target_voltage = clip(v, voltage_limit); }

    void set_voltage_limit(float limit) { voltage_limit = clip(limit, 0.f, 1.f); voltage_limit_changed = true; pos_ctrl.set_limit(voltage_limit); }

    void apply_impulse(float value, unsigned duration) { imp_ctrl.value = value; imp_ctrl.duration = duration; }

    void set_pwm_frequency  (float f_hz) { target_period_us  = 1000ul*1000.0/f_hz; }
    uint16_t get_pwm_period (void) const { return target_period_us; }

    void set_direction(int16_t dir) { direction = dir; }
    void set_scalefactor(float scf) { scalefactor = scf; }

    void set_offset(float o) { offset = o; }
    void add_offset(float o) { offset += o; }
    float get_offset(void) const { return offset; }

    void set_zero_position(void) { offset -= data.position; }

    void set_disable_position_limits(float lo, float hi) { lim_disable_lo = lo; lim_disable_hi = hi; }

    void set_ext_sensor_readout(bool enable) { read_external_sensor = enable; }

    void execute_controller(void)
    {
        /* update and reset if mode changes */
        if (controller != targetctrl) {
            csl_ctrl.reset(data.position);
            pos_ctrl.reset();
            imp_ctrl.reset();
            controller = targetctrl;
        }

        if (not in_range(data.position, lim_disable_lo, lim_disable_hi))
            disable();

        switch (controller) {
        case Controller_t::position: set_target_voltage( pos_ctrl.step(data.position) ); break;
        case Controller_t::csl     : set_target_voltage( csl_ctrl.step(data.position) ); break;
        case Controller_t::impulse : set_target_voltage( imp_ctrl.step()              ); break;
        case Controller_t::none    : set_target_voltage( 0.f                          ); break;
        default:
            assert(false);
        }
    }

private:

    /** TODO: enqueue sync bytes and checksum could be done by someone else since each package is affected */

    void enqueue_command_data_request() {
        data.output_voltage = .0;
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

    void enqueue_command_set_voltage(float voltage) {
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

    void enqueue_command_set_voltage_and_period(float voltage, uint16_t period) {
        data.output_voltage = voltage;
        voltage *= direction; // correct direction
        com.enqueue_sync_bytes(0xFF);
        if (voltage >= 0.0) {
            com.enqueue_byte(0xD0);
        } else {
            com.enqueue_byte(0xD1);
        }
        com.enqueue_byte(motor_id);
        com.enqueue_word(period);
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

    void enqueue_command_send_raw_data(void) {
        com.enqueue_sync_bytes(0xFF);
        com.enqueue_byte(0x55);
        com.enqueue_byte(motor_id);
        com.enqueue_byte(data.num_sendbytes);
        for (uint8_t i = 0; i < data.num_sendbytes; ++i)
            com.enqueue_byte(data.raw_send[i]);
        com.enqueue_checksum();
    }

    std::size_t send_motor_command(void) {
        enqueue_command_set_voltage_limit();
        switch(controller) {
        case Controller_t::none:
            enqueue_command_data_request();
            break;
        case Controller_t::voltage:
        case Controller_t::position:
        case Controller_t::csl:
        case Controller_t::impulse:
            if (0 != target_period_us) enqueue_command_set_voltage_and_period(target_voltage, target_period_us);
            else enqueue_command_set_voltage(target_voltage);
            break;
        case Controller_t::send_raw:
            enqueue_command_send_raw_data();
            break;
        default: assertion(false, "Unknown controller type %u", controller); break;
        }

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

        data.statistics.update(t_us, t_us >= timeout_us, !is_data_valid());
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
                        const uint8_t mid = com.get_byte();
                        if (mid == motor_id) {
                            data.position_0     = data.position;
                            data.position       = uint16_to_sc(com.get_word()) * direction * scalefactor + offset;
                            data.current        = ((int16_t) com.get_word()) * unit_A;
                            data.velocity_lpf   = int16_to_sc(com.get_word()) * direction * scalefactor;
                            data.velocity       = (data.position - data.position_0) * inv_dt;
                            data.voltage_supply = com.get_word() * unit_V;
                            data.temperature    = static_cast<int16_t>(com.get_word()) * unit_C;
                            com.get_byte(); /* eat checksum */
                        }
                        syncstate = (motor_id == mid and com.is_checksum_ok()) ? completed : invalid;
                        is_responding = (syncstate == completed);
                        return true;
                    }
                    return false;

                case 0x56: /* raw data response */
                    if (com.size() > 2u && com.size() > com.look_ahead(2) + 3u) { // N raw bytes + (cmd + id + Nbytes + chksum)
                        com.get_byte(); /* eat command byte */
                        const uint8_t mid = com.get_byte();
                        if (mid == motor_id) {
                            const uint8_t len = com.get_byte();
                            for (uint8_t i = 0; i < len; ++i)
                                data.raw_recv[i] = com.get_byte();
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
