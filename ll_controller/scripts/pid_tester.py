# import sys

from ll_controller.customPID import PID

import time

target_velocity = 100
speed_pid = PID(Kp=1, Ki=0.0, Kd=0.0, SP=target_velocity, name='Speed_PID')  # SP is in kmph


current_velocity = 0
for i in range(0, target_velocity):
    current_velocity = i

    generated_velocity = speed_pid.get_output(target_velocity, current_velocity)
    # my_vel = 1 * (target_velocity - current_velocity)
    print("curr_vel : {}, target_vel : {}, ------ calculated_vel = {}".format(current_velocity, target_velocity, generated_velocity))
    time.sleep(0.1)