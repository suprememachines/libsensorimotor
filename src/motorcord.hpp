#ifndef SUPREME_MOTORCORD_HPP
#define SUPREME_MOTORCORD_HPP

#include "sensorimotor.hpp"
#include "communication_ctrl.hpp"

namespace supreme {

class motorcord {

    const uint8_t max_boards = 128;

    std::size_t cyclecounter = 0;

    communication_controller<1000000>  com;

    std::vector<sensorimotor> motors;

    bool rescan_for_motors = true;
    std::size_t num_active_motors = 0;

    bool verbose;

public:
    motorcord(uint8_t number_of_boards, float update_rate_Hz, bool verbose = true)
    : com(), motors(), verbose(verbose)
    {
        assert(number_of_boards <= max_boards);
        for (uint8_t id = 0; id < number_of_boards; ++id)
            motors.emplace_back(id, com, update_rate_Hz);
    }

    ~motorcord() {
        sts_msg("Disabling motors");
        disable_all();
        execute_cycle();
        sts_msg("All motors disabled.");
    }

    void disable_all(void) { for (auto &m : motors) m.disable(); }

          sensorimotor& operator[] (std::size_t index)       { return motors.at(index); }
    const sensorimotor& operator[] (std::size_t index) const { return motors.at(index); }

    std::size_t size() const { return motors.size(); }

    void reset_statistics(void) { for (auto& m : motors) m.reset_statistics(); }
    void rescan(void) { reset_statistics(); rescan_for_motors = true; }


    void execute_cycle()
    {
        if (rescan_for_motors) scan_for_motors();

        if (num_active_motors == 0) {
            com.sleep_s(1);
            rescan_for_motors = true;
        }

        if (verbose)
            printf("%02lu | ", cyclecounter % 100);

        ++cyclecounter;

        unsigned errors = 0;
        unsigned timeouts = 0;
        for (auto& m : motors) if (m.is_active()) {
            auto const& stats = m.execute_cycle();
            errors += stats.errors;
            timeouts += stats.timeouts;

            if (verbose) {
                if (!stats.faulted)
                    printf("%u:%02u ", m.get_id(), stats.response_time_us);
                else
                    printf("%u:_%u ", m.get_id(), m.get_syncstate());
            }
        }

        if (verbose)
            printf("| e=%u t=%u\n", errors, timeouts);

        for (auto& m : motors) if (m.is_active())
            m.execute_controller();
    }


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

};

} /* namespace supreme */

#endif /* SUPREME_MOTORCORD_HPP */
