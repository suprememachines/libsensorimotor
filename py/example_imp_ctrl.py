#!/usr/bin/python

"""
    Example Code for Impulse Control
    Supreme Sensorimotor Library
    Matthias Kubisch
    Jetpack Cognition Lab
    Last Update: January 22nd 2020

    +---------------------------------------------------------+
    | WARNING: This example demonstrates the impulse control  |
    | method to be used with SOLENOIDS and should NOT be used |
    | for SERVO MOTORS with end-stops. ~~~~~~~~~~~~~~~~~~~~~~ |
    +---------------------------------------------------------+
"""

from src.sensorimotor import Sensorimotor
from time import sleep

# example beat 0
tst0 = [ ([0.6, 0.0, 0.0], 1./4) # bar 1
       , ([0.0, 0.0, 0.6], 1./4)
       , ([0.0, 0.6, 0.0], 1./4)
       , ([0.0, 0.0, 0.6], 1./4)
       , ([0.6, 0.0, 0.0], 1./8) # bar 2
       , ([0.0, 0.0, 0.6], 1./4)
       , ([0.6, 0.0, 0.0], 1./8)
       , ([0.0, 0.6, 0.0], 1./4)
       , ([0.0, 0.0, 0.6], 1./4)
       ]

# example beat 1
tst1 = [ ([0.6, 0.0, 0.6], 1./4)
       , ([0.0, 0.0, 0.6], 1./4)
       , ([0.0, 0.6, 0.6], 1./4)
       , ([0.0, 0.6, 0.6], 1./4)
       ]

def main():
    motors = Sensorimotor(number_of_motors = 3, verbose = False)

    try:
        # checking for motors
        N = motors.ping()
        print("Found {0} sensorimotors.".format(N))
        sleep(1.0)

        # TODO (for USER):
        # Set voltage limits according to your supply voltage and desired max. motor speed.
        # A value of e.g. 0.25 means the motor is driven with PWM of 25% duty cycle.
        # So if your supply voltage is e.g. 12V the motor is driven with 3V (on average).
        # Default is 0, so your motors will not move until this value is set.
        #motors.set_voltage_limit([0.25, 0.25, 0.25])

        # starting motorcord
        motors.start()

        beat = tst0
        duration = [5, 5, 5] # pulse length in number of steps, e.g. 5x10ms = 50ms

        while(motors.running()):
            # LOOP BEGIN

            print("tak")
            for b in beat:
                motors.apply_impulse(b[0], duration)
                sleep(b[1])

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
