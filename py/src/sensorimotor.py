"""

 +-----------------------------------+
 |  Supreme Sensorimotor Library     |
 |  Matthias Kubisch                 |
 |  Jetpack Cognition Lab            |
 |  kubisch@informatik.hu-berlin.de  |
 |  Last Update: January 22nd 2020   |
 +-----------------------------------+


"""

#TODO: let the library drive each motor in a different control mode

from ctypes import cdll
from ctypes import c_int, c_uint8, c_float, c_void_p, c_bool

import threading

# Loading shared Library
lib = cdll.LoadLibrary('../bin/libsensorimotor.so')


class UX0Data(object):

    def __init__(self, num_motors):
        self.pos = [0.0] * num_motors
        self.vel = [0.0] * num_motors
        self.cur = [0.0] * num_motors
        self.vol = [0.0] * num_motors
        self.tem = [0.0] * num_motors

        self.raw_send = [[0]*256] * num_motors
        self.raw_recv = [[0]*256] * num_motors


class Sensorimotor(object):

    def __init__(self, number_of_motors = 128, update_rate_Hz = 100, verbose = True):
        set_types()

        # preparing interface and motors
        self.obj = lib.sensorimotor_new(c_uint8(number_of_motors), c_float(update_rate_Hz), c_bool(verbose))
        self.number_of_motors = number_of_motors
        self.data = UX0Data(number_of_motors)

        # creating thread
        self.stop_t = threading.Event()
        self.loop_t = threading.Thread(target=self.__execute_cycle)


    def __del__(self):
        n = lib.sensorimotor_del(self.obj)
        if n == 0:
            print("Motor cord successfully stopped.")


    def start(self):
        self.stop_t.clear()
        self.loop_t.start()


    def stop(self):
        self.stop_t.set()
        self.loop_t.join()


    def running(self):
        return self.loop_t.is_alive()


    def __check_args(self, li):
        assert len(li) == self.number_of_motors, "Invalid number of list elements."

    def __check_id(self, id):
        assert 0 <= id < self.number_of_motors, "Invalid motor id."

    def set_position(self, positions):
        self.__check_args(positions)
        target_position = list(positions)
        carray = (c_float * len(target_position))(*target_position)
        n = lib.sensorimotor_set_position(self.obj, carray, c_uint8(len(carray)))


    def set_pos_ctrl_params(self, motor_id, Kp = 0.0, Ki = 0.0, Kd = 0.0, deadband = 0.0, pulse_threshold = 0.0):
        self.__check_id(motor_id)
        params = [Kp, Ki, Kd, deadband, pulse_threshold]
        carray = (c_float * len(params))(*params)
        n = lib.sensorimotor_set_pos_ctrl_params(self.obj, c_uint8(motor_id), carray, c_uint8(len(carray)))


    def set_voltage_limit(self, limits):
        self.__check_args(limits)
        target_limits = list(limits)
        carray = (c_float * len(target_limits))(*target_limits)
        n = lib.sensorimotor_set_voltage_limit(self.obj, carray, c_uint8(len(carray)))


    def apply_impulse(self, values, durations):
        self.__check_args(values)
        self.__check_args(durations)
        target_val = list(values)
        target_dur = list(durations)
        _val = (c_float * len(target_val))(*target_val)
        _dur = (c_float * len(target_dur))(*target_dur)
        n = lib.sensorimotor_apply_impulse(self.obj, _val, _dur, c_uint8(len(_val)))


    # todo split to set and apply, or set flag if changed or sth.
    def set_raw_data_send(self, motor_id, data):
        assert len(data) <= len(self.data.raw_send[motor_id]), "Invalid number of list elements."
        self.__check_id(motor_id)
        self.data.raw_send[motor_id] = list(data)
        _raw = (c_uint8 * len(self.data.raw_send[motor_id]))(*self.data.raw_send[motor_id])
        n = lib.sensorimotor_set_raw_data_send(self.obj, c_uint8(motor_id), _raw, c_uint8(len(_raw)))


    def __get_motor_data(self): #TODO use struct?
        N = len(self.data.pos)
        _pos = (c_float * N)(*self.data.pos)
        _vel = (c_float * N)(*self.data.vel)
        _cur = (c_float * N)(*self.data.cur)
        _vol = (c_float * N)(*self.data.vol)
        _tem = (c_float * N)(*self.data.tem)

        n = lib.sensorimotor_get_motor_data(self.obj, _pos, _vel, _cur, _vol, _tem, c_uint8(len(_pos)))
        self.data.pos = list(_pos)
        self.data.vel = list(_vel)
        self.data.cur = list(_cur)
        self.data.vol = list(_vol)
        self.data.tem = list(_tem)


    def __execute_cycle(self):
        while(not self.stop_t.is_set()):
            n = lib.sensorimotor_execute_cycle(self.obj)
            if (n < 0):
                print("Error: Assertion in Libsensorimotor (C/C++).")
                break
            self.__get_motor_data()


    def ping(self):
        n = lib.sensorimotor_ping(self.obj)
        return n


    def get_position(self, index=None):
        return list(self.data.pos) if index is None else self.data.pos[index]

    def get_velocity(self, index=None):
        return list(self.data.vel) if index is None else self.data.vel[index]

    def get_current(self, index=None):
        return list(self.data.cur) if index is None else self.data.cur[index]

    def get_voltage(self, index=None):
        return list(self.data.vol) if index is None else self.data.vol[index]

    def get_temperature(self, index=None):
        return list(self.data.tem) if index is None else self.data.tem[index]

    # todo split to fetch and get
    def get_raw_data_recv(self, motor_id, Nbytes = 256):
        assert 0 <= motor_id < self.number_of_motors, "Invalid motor id."
        N = len(self.data.raw_recv[motor_id])
        assert Nbytes <= N, "Invalid number of bytes requested."
        _raw = (c_uint8 * N)(*self.data.raw_recv[motor_id])
        lib.sensorimotor_get_raw_data_recv(self.obj, motor_id, _raw, c_uint8(Nbytes))
        self.data.raw_recv[motor_id] = list(_raw)
        return self.data.raw_recv[motor_id][:Nbytes]

# END CLASS Sensorimotor


def set_types():
    lib.sensorimotor_new.argtypes = [c_uint8, c_float, c_bool]
    lib.sensorimotor_new.restype = c_void_p

    lib.sensorimotor_ping.argtypes = [c_void_p]
    lib.sensorimotor_ping.restype = c_int

    lib.sensorimotor_del.argtypes = [c_void_p]
    lib.sensorimotor_del.restype = c_int

    lib.sensorimotor_execute_cycle.argtypes = [c_void_p]
    lib.sensorimotor_execute_cycle.restype = c_int

    lib.sensorimotor_set_position.argtypes = [c_void_p, c_void_p, c_uint8]
    lib.sensorimotor_set_position.restype = c_int

    lib.sensorimotor_set_pos_ctrl_params.argtypes = [c_void_p, c_uint8, c_void_p, c_uint8]
    lib.sensorimotor_set_pos_ctrl_params.restype = c_int

    lib.sensorimotor_set_voltage_limit.argtypes = [c_void_p, c_void_p, c_uint8]
    lib.sensorimotor_set_voltage_limit.restype = c_int

    lib.sensorimotor_apply_impulse.argtypes = [c_void_p, c_void_p, c_void_p, c_uint8]
    lib.sensorimotor_apply_impulse.restype = c_int

    lib.sensorimotor_get_motor_data.argtypes = [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p, c_void_p,c_uint8]
    lib.sensorimotor_get_motor_data.restype = c_int

    lib.sensorimotor_set_raw_data_send.argtypes = [c_void_p, c_uint8, c_void_p, c_uint8]
    lib.sensorimotor_set_raw_data_send.restype = c_int

    lib.sensorimotor_get_raw_data_recv.argtypes = [c_void_p, c_uint8, c_void_p, c_uint8]
    lib.sensorimotor_get_raw_data_recv.restype = c_int
