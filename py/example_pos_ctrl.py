#!/usr/bin/python

"""
    Example Code for Position Control
    Supreme Sensorimotor Library
    Matthias Kubisch
    Jetpack Cognition Lab
    Last Update: January 22nd 2020

"""


from src.sensorimotor import Sensorimotor
from time import sleep

def print_vec(data):
    print(''.join('{0: .3f} '.format(k) for k in data))

def main():
    motors = Sensorimotor(number_of_motors=2, verbose=False)

    try:
        # checking for motors
        N = motors.ping()
        print("Found {0} sensorimotors.".format(N))
        sleep(1.0)

        # TODO (for USER):
        # Set voltage limits according to your supply voltage and desired max. motor speed.
        # A value of e.g. 0.1 means the motor is driven with PWM of 10% duty cycle.
        # So if your supply voltage is e.g. 12V the motor is driven with 1.2V (on average).
        # Default is 0, so your motors will not move until this value is set.
        motors.set_voltage_limit([0.1, 0.1])

        # starting motorcord
        motors.start()

        # TODO (for USER):
        # Set this parameters according to your desired motor positon control behaviour.
        # Read the documentation for explanation of the specific parameters.
        motors.set_pos_ctrl_params(motor_id=1, Kp=1.0, Ki=0.0, Kd=0.0, deadband=0.00, pulse_threshold=0.00)

        while(motors.running()):
            # LOOP BEGIN

            motors.set_position([0.0, 0.0])
            print_vec(motors.get_position())
            #print_vec(motors.get_velocity())

            sleep(0.01)

            # LOOP END


    except (KeyboardInterrupt, SystemExit):
        # stopping motor cord
        print("\rAborted, stopping motors")
        motors.stop()

    except:
        # Script crashed?
        print("\rException thrown, stopping motors")
        motors.stop()
        raise

    print("____\nDONE.")


if __name__ == "__main__":
    main()
