#!/usr/bin/python

from src.sensorimotor import Sensorimotor
from time import sleep

def print_position(data):
    print(''.join('{0: .2f} '.format(k) for k in data))

def main():
    motors = Sensorimotor(number_of_motors = 3, verbose = False)
    
    # checking for motors
    N = motors.ping()
    print("Found {0} sensorimotors.".format(N))
    sleep(1.0)
        
    # starting motorcord
    motors.start()

    try:

        while(True):
            # USER CODE HERE BEGIN
         
            motors.set_position([0.2, 0.2, 0.2])
            print_position(motors.get_position())
            sleep(1.0)
            motors.set_position([0.0, 0.0, 0.00])
            print_position(motors.get_position())
            sleep(1.0)

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
