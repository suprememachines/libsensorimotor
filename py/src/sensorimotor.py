"""

 +-----------------------------------+
 |  Sensorimotor Library             |
 |  Matthias Kubisch                 |
 |  kubisch@informatik.hu-berlin.de  |
 |  October 2018                     |
 +-----------------------------------+


"""

#TODO: let the library drive each motor in a different control mode

from ctypes import cdll
from ctypes import c_int, c_uint, c_double, c_char_p, c_void_p, c_bool

import threading
from time import sleep

# Loading shared Library
lib = cdll.LoadLibrary('../bin/libsensorimotor.so')


class Sensorimotor(object):
    def __init__(self, number_of_motors = 127, update_rate_Hz = 100, verbose = True):

        set_types()

        # preparing interface and motors
        self.obj = lib.sensorimotor_new(c_uint(number_of_motors), c_double(update_rate_Hz), c_bool(verbose))
        self.number_of_motors = number_of_motors


        # creating thread
        self.stop_t = threading.Event()
        self.loop_t = threading.Thread(target=self.__execute_cycle)

        #self.target_position = [0.0] * self.number_of_motors
        self.motor_data = [0.0] * self.number_of_motors #TODO currently only positions


    def __del__(self):
         lib.sensorimotor_del(self.obj)


    def start(self):
        self.stop_t.clear()
        self.loop_t.start()


    def stop(self):
        self.stop_t.set()
        self.loop_t.join()


    def set_position(self, positions):
        assert len(positions) <= self.number_of_motors
        target_position = list(positions)
        carray = (c_double * len(target_position))(*target_position)
        n = lib.sensorimotor_set_position(self.obj, carray, c_uint(len(carray)))

    def set_voltage_limit(self, limits):
        assert len(limits) <= self.number_of_motors
        target_limits = list(limits)
        carray = (c_double * len(target_limits))(*target_limits)
        n = lib.sensorimotor_set_voltage_limit(self.obj, carray, c_uint(len(carray)))

    def apply_impulse(self, impulses):
        assert len(impulses) <= self.number_of_motors
        target_impulses = list(impulses)
        carray = (c_double * len(target_impulses))(*target_impulses)
        n = lib.sensorimotor_apply_impulse(self.obj, carray, c_uint(len(carray)))

    def __get_motor_data(self):
        carray = (c_double * len(self.motor_data))(*self.motor_data)
        n = lib.sensorimotor_get_motor_data(self.obj, carray, c_uint(len(carray)))
        self.motor_data = list(carray)
        

    def get_position(self):
        return self.motor_data


    def __execute_cycle(self):
        while(not self.stop_t.is_set()):
            n = lib.sensorimotor_execute_cycle(self.obj)
            self.__get_motor_data()
            if (n < 0):
                print("unexpected stop")
                stop(self)


    def ping(self):
        n = lib.sensorimotor_ping(self.obj)
        return n



def set_types():
    lib.sensorimotor_new.argtypes = [c_uint, c_double, c_bool]
    lib.sensorimotor_new.restype = c_void_p

    lib.sensorimotor_ping.argtypes = [c_void_p]
    lib.sensorimotor_ping.restype = c_int

    lib.sensorimotor_del.argtypes = [c_void_p]
    lib.sensorimotor_del.restype = None

    lib.sensorimotor_execute_cycle.argtypes = [c_void_p]
    lib.sensorimotor_execute_cycle.restype = c_int

    lib.sensorimotor_set_position.argtypes = [c_void_p, c_void_p, c_uint]
    lib.sensorimotor_set_position.restype = c_int

    lib.sensorimotor_set_voltage_limit.argtypes = [c_void_p, c_void_p, c_uint]
    lib.sensorimotor_set_voltage_limit.restype = c_int

    lib.sensorimotor_apply_impulse.argtypes = [c_void_p, c_void_p, c_uint]
    lib.sensorimotor_apply_impulse.restype = c_int

    lib.sensorimotor_get_motor_data.argtypes = [c_void_p, c_void_p, c_uint]
    lib.sensorimotor_get_motor_data.restype = c_int


