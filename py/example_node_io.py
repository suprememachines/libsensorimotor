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


def sawtooth(i):
    return min(255,abs(i%511 - 255))


def main():
    cord = Sensorimotor(number_of_motors=6, verbose=False, update_rate_Hz = 100)

    try:
        # checking for motors
        N = cord.ping()
        print("Found {0} sensorimotors.".format(N))
        sleep(0.25)

        # starting motorcord
        cord.start()
        i = 0
        while(cord.running()):
            # LOOP BEGIN

            f = sawtooth(i)

            mot = [0,0,f] # esc, servopos, light
            for b in range(cord.number_of_motors):
                cord.set_raw_data_send(b, mot)

            x = cord.get_raw_data_recv(0, 11)
            print(x)
            sleep(0.01) # todo replace by framesync
            i += 1
            # LOOP END


    except (KeyboardInterrupt, SystemExit):
        # stopping motor cord
        print("\rAborted, stopping motors")
        cord.stop()

    except:
        # Script crashed?
        print("\rException thrown, stopping cord.")
        cord.stop()
        raise

    print("____\nDONE.")


if __name__ == "__main__":
    main()
