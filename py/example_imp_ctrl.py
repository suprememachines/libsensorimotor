#!/usr/bin/python

"""
   +---------------------------------------------------------+
   | Warning: This example demonstrates the impulse control  |
   | method to be used with solenoids and should NOT be used |
   | for servo motors with end-stops. ~~~~~~~~~~~~~~~~~~~~~~ |
   +---------------------------------------------------------+
"""

from src.sensorimotor import Sensorimotor
from time import sleep

def print_position(data):
    print(''.join('{0: .2f} '.format(k) for k in data))


tst0 = [ ([0.0, 0.6, 0.0], 0.20)
       , ([0.0, 0.0, 0.6], 0.20)
       , ([0.6, 0.0, 0.0], 0.20)
       , ([0.0, 0.0, 0.6], 0.20)
       , ([0.0, 0.6, 0.0], 0.10)
       , ([0.0, 0.0, 0.6], 0.20)
       , ([0.0, 0.6, 0.0], 0.10)
       , ([0.6, 0.0, 0.0], 0.20)
       , ([0.0, 0.0, 0.6], 0.20)
       ]

tst1 = [ ([0.0, 0.6, 0.0], 0.20)
       , ([0.0, 0.0, 0.6], 0.20)
       , ([0.6, 0.0, 0.0], 0.20)
       , ([0.0, 0.0, 0.6], 0.20)
       ]


def main():
    motors = Sensorimotor(number_of_motors = 3, verbose = False)

    # checking for motors
    N = motors.ping()
    print("Found {0} sensorimotors.".format(N))
    sleep(1.0)

    # starting motorcord
    motors.set_voltage_limit([0.6, 0.6, 0.6])
    motors.start()

    beat = tst0

    try:

        while(True):
            # USER CODE HERE BEGIN

            print("tak")
            for b in beat:
                motors.apply_impulse(b[0])
                sleep(b[1]*1.2)

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


if __name__ == "__main__": main()
