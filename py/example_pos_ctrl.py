#!/usr/bin/python

from src.sensorimotor import Sensorimotor
from time import sleep

def print_position(data):
    print(''.join('{0: .2f} '.format(k) for k in data))

def main():
    motors = Sensorimotor(number_of_motors=2, verbose=True)

    # checking for motors
    N = motors.ping()
    print("Found {0} sensorimotors.".format(N))
    sleep(1.0)

    # starting motorcord
    motors.set_voltage_limit([0.1, 0.1])
    motors.start()

    try:

        while(True):
            # USER CODE HERE BEGIN

            motors.set_position([0.2, 0.2])
            #print_position(motors.get_position())
            sleep(0.25)
            motors.set_position([0.2, -0.2])
            #print_position(motors.get_position())
            sleep(0.25)
            motors.set_position([-0.2, -0.2])
            #print_position(motors.get_position())
            sleep(0.25)
            motors.set_position([-0.2, +0.2])
            #print_position(motors.get_position())
            sleep(0.25)

            # USER CODE HERE END


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
