#ifndef SUPREME_MOTORCORD_HPP
#define SUPREME_MOTORCORD_HPP

#include "sensorimotor.hpp"
#include "communication_ctrl.hpp"

namespace supreme {

/** TODOs:
    + make this class thread-safe
    + remove print statements -> move to visualization class
    + remove everything system-specific
    + replace std::vector by static vector
*/

class motorcord {

    const uint8_t max_boards = 128;
    uint8_t selected = 0;

    std::size_t cyclecounter = 0;

    communication_controller<1000000>  com;

    std::vector<sensorimotor> motors;

    bool rescan_for_motors = true;
    std::size_t num_active_motors = 0;

    bool verbose;

public:
    motorcord(uint8_t number_of_boards, bool verbose = true)
    : com(), motors(), verbose(verbose)
    {
        assert(number_of_boards <= max_boards);
        for (uint8_t id = 0; id < number_of_boards; ++id)
            motors.emplace_back(id, com);
    }

    ~motorcord() {
        sts_msg("Disabling motors");
        for (auto &m : motors) m.disable();
        execute_cycle();
        sts_msg("All motors disabled.");
    }

          sensorimotor& operator[] (std::size_t index)       { return motors.at(index); }
    const sensorimotor& operator[] (std::size_t index) const { return motors.at(index); }

    std::size_t size() const { return motors.size(); }

    void reset_statistics(void) { for (auto& m : motors) m.reset_statistics(); }
    void rescan(void) { rescan_for_motors = true; }

    //void set_direction(std::size_t index, int16_t dir) { motors.at(index).set_direction(dir); }

    void execute_cycle()
    {
        if (rescan_for_motors) scan_for_motors();
        if (num_active_motors == 0) {
            com.sleep_s(1);
            rescan_for_motors = true;
        }

        if (verbose)
            printf("[%u] %02lu | ", selected, cyclecounter % 100);

        ++cyclecounter;

        unsigned errors = 0;
        for (auto& m : motors) if (m.is_active()) {
            auto stats = m.execute_cycle();
            errors += stats.errors;

            if (verbose) {
                if (!stats.faulted)
                    printf("%u:%02u ", m.get_id(), stats.response_time_us);
                else
                    printf("%u:_%u ", m.get_id(), m.get_syncstate());
            }
        }

        //printf("| e=%u, t=%2uu(%2u)\n", errors, (unsigned) round(avg_resp_time_us*byte_delay_us), max_resp_time_us);
        if (verbose)
            printf("| e=%u\n", errors);

        for (auto& m : motors) if (m.is_active())
            m.execute_controller();
    }

    void toggle_led(std::size_t index) {
        if (index < motors.size())
            motors.at(index).toggle_led();
        else wrn_msg("No such board to toggle led.");
    }

    void toggle_request   () { motors.at(selected).toggle_request(); }

    unsigned scan_for_motors() {
        rescan_for_motors = false;
        printf("scanning: ");
        num_active_motors = 0;
        for (auto& m : motors) {
            if (m.ping()) {
                ++num_active_motors;
                printf("%02u ",m.get_id());
            }
        }
        printf("\n");
        return num_active_motors;
    }

    void set_target_position(double p) { motors.at(selected).set_target_position(p); }
    void set_target_voltage (double v) { motors.at(selected).set_target_voltage (v); }

    void set_position_ctrl() { motors.at(selected).set_controller_type(sensorimotor::Controller_t::position); }
    void set_csl_ctrl()      { motors.at(selected).set_controller_type(sensorimotor::Controller_t::csl     ); }
    void set_voltage_ctrl()  { motors.at(selected).set_controller_type(sensorimotor::Controller_t::voltage ); }

    uint8_t get_id() const { return selected; }

    void set_id(uint8_t id) { if (id < motors.size()) selected = id; }
};

} /* namespace supreme */

#endif /* SUPREME_MOTORCORD_HPP */
