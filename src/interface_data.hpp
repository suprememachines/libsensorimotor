#ifndef SUPREME_INTERFACE_DATA_HPP
#define SUPREME_INTERFACE_DATA_HPP

namespace supreme {

    struct interface_data {
        double output_voltage;
        double position;
        double velocity;
        double current;
        double voltage_backemf;
        double voltage_supply;
        double temperature;
    };

}

#endif /* SUPREME_INTERFACE_DATA_HPP */
