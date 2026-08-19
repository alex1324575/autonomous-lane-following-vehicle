# PID - By: haaris - Thu Apr 10 2025
#
# PID should be calculated during Timer-based interrupts.

import const
from time import ticks_us, ticks_diff
from math import isnan

class PID:
    # == Initializer == #
    def __init__(self, p=0, i=0, d=0, i_max=0, i_resetting=False):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)

        self._integrator = 0
        self._i_max = i_max
        self._i_resetting = i_resetting

        self._prev_time = 0
        self._any_nans = True
        self._d_index_start = 0
        self._d_index_end   = 0

        self._last_errors = []
        self._last_times = []
        self._D_prev = float("nan")

        for i in range(const.D_BUFF_SIZE):
            self._last_errors.append(float("nan"))
            self._last_times.append(float("nan"))


    # == Method Functions == #
    def get_pid(self, error):
        self.__update_buffs_and_indices(error)

        P = self.calc_P()
        I = self.calc_I()
        D = self.calc_D()

        sum = P + I + D
        return sum

    def __update_buffs_and_indices(self, error):
        if (self._i_resetting):
            self.__reset_I()
            return

        # If the buffs are not full yet, just add the values and return.
        if isnan(self._last_times[self._d_index_end]):
            self._last_errors[self._d_index_end] = error
            self._last_times[self._d_index_end] = ticks_us()

            self._d_index_end = (self._d_index_end + 1) % const.D_BUFF_SIZE
            return


        ## Ensure that the PID only updates previous times and errors if ##
        ## there are at least TIME_DELTA microseconds between one another. ##
        self._any_nans = False
        temp = ticks_us()
        diff = ticks_diff(temp, self._last_times[self._d_index_end])

        # If less than TIME_DELTA microseconds have passed since last measurement,
        # don't update!
        if (diff < const.TIME_DELTA): return

        # If more than TIME_DELTA microseconds have passed since last measurement,
        # continue.
        new_end_index   = (self._d_index_end + 1)   % const.D_BUFF_SIZE
        new_start_index = (self._d_index_start + 1) % const.D_BUFF_SIZE

        self._last_errors[self._d_index_end] = error
        self._last_times[self._d_index_end] = temp

        self._d_index_start = new_start_index
        self._d_index_end = new_end_index

    def calc_P(self):
        if self._any_nans: return 0

        e_avg = 0
        for x in self._last_errors:
            e_avg += x
        e_avg /= const.D_BUFF_SIZE

        P = e_avg * self._kp
        return P


    def calc_D(self):
        if not self._kd   : return 0
        if self._any_nans : return 0

        old_n_index   = (self._d_index_end - 1) % const.D_BUFF_SIZE
        old_n_1_index = (self._d_index_end - 2) % const.D_BUFF_SIZE
        old_n_2_index = (self._d_index_end - 3) % const.D_BUFF_SIZE
        old_n_3_index = (self._d_index_end - 4) % const.D_BUFF_SIZE

        error_n   = self._last_errors[old_n_index]      # e[n]
        error_n_1 = self._last_errors[old_n_1_index]    # e[n-1]
        error_n_2 = self._last_errors[old_n_2_index]    # e[n-2]
        error_n_3 = self._last_errors[old_n_3_index]    # e[n-3]

        T_n   = self._last_times[old_n_index]
        T_n_1 = self._last_times[old_n_1_index]
        T_n_2 = self._last_times[old_n_2_index]
        T_n_3 = self._last_times[old_n_3_index]

        delta_T_0_to_3 = ticks_diff(T_n, T_n_3)   / const.US_TO_SEC_MULTPLIER
        delta_T_1_to_2 = ticks_diff(T_n_1, T_n_2) / const.US_TO_SEC_MULTPLIER

        D_1 = (error_n - error_n_3)   / delta_T_0_to_3
        D_2 = (error_n_1 - error_n_2) / delta_T_1_to_2

        D = (D_1 + D_2) / 2

        self._D_prev = D
        D *= self._kd

        return D


    def calc_I(self):
        if not self._ki   : return 0
        if self._any_nans : return 0

        old_end_index   = (self._d_index_end - 1) % const.D_BUFF_SIZE
        older_end_index = (self._d_index_end - 2) % const.D_BUFF_SIZE

        e_curr = self._last_errors[old_end_index]      # e[n]
        e_prev = self._last_errors[older_end_index]    # e[n-1]

        t_curr = self._last_times[old_end_index]
        t_prev = self._last_times[older_end_index]

        delta_t = ticks_diff(t_curr, t_prev) / const.US_TO_SEC_MULTPLIER

        area = (e_curr - e_prev) / 2
        area *= delta_t
        area *= self._ki

        self._integrator += area
        if (self._integrator < -self._i_max): self._integrator = -self._i_max
        if (self._integrator >  self._i_max): self._integrator =  self._i_max

        I = self._integrator
        return I


    def __reset_I(self):
        self._integrator = 0

        for i in range(const.D_BUFF_SIZE):
            self._last_errors[i](float("nan"))
            self._last_times[i](float("nan"))

        self._any_nans = True
        self._d_index_start = 0
        self._d_index_end = 0

