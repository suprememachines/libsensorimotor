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
    cord = Sensorimotor(number_of_motors=6, verbose=False, update_rate_Hz = 50)

    try:
        # checking for motors
        N = cord.ping()
        print("Found {0} sensorimotors.".format(N))
        sleep(0.25)

        # starting motorcord
        cord.start()
        x = cord.get_raw_data_recv(0, 11)
        while(cord.running()):
            # LOOP BEGIN

            mot = [x[1],x[2],x[3],x[4]]
            cord.set_raw_data_send(0, mot)
            x = cord.get_raw_data_recv(0, 11)
            print(x)
            sleep(0.01)

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
