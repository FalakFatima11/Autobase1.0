#!/usr/bin/env python
'''
Listens to cmd_vel and ll_state
Converts desired velocity to throttle and break values, publishes to ll_cmd
'''
import rospy
import time
import math
import datetime

from threading import Thread, Lock

from communication.msg import CarState
from communication.msg import CarControls
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from ll_controller.customPID import PID

class LL_controller():
    def __init__(self, params):
        self.target_velocity = 0
        self.target_steering_angle = 0

        self.emergency_braking = False

        self.current_velocity = 0
        self.driving_mode = "auto"

        self.generated_ll_cmd = CarControls()

        self.ll_state_ts = None
        self.cmd_vel_ts = None

        self.ts_delay_threshold = 1
        self.Kp_th = params['kp_th']
        self.Ki_th = params['ki_th']
        self.Kd_th = params['kd_th']
        self.throttle_pid = PID(Kp=self.Kp_th, Ki=self.Ki_th, Kd=self.Kd_th, SP=self.target_velocity, name='Throttle_PID')  # SP is in kmph

        self.Kp_br = params['kp_br']
        self.Ki_br = params['ki_br']
        self.Kd_br = params['kd_br']
        # self.Kd_brake = -2.0
        self.brake_pid = PID(Kp=self.Kp_br, Ki=self.Ki_br, Kd=self.Kd_br, SP=self.target_velocity, name='Brake_PID')  # SP is in kmph

        self.throttle_limit = params['throttle_limit']
        self.brake_limit = params['brake_limit']

        
        self.initSubscribers()
        self.initPublishers()

        self.control_mutex = Lock()

        # self.logs = open("/home/ysk/brake_logs.txt","w+")

        self.rate = rospy.Rate(10)

        self.log_flags = {
                          'not_auto_mode' : False,
                          'no_cmd_vel': False, 
                          'no_ll_state': False,
                          'timeout_cmd_vel': False,
                          'timeout_ll_state': False
                         }

    def initSubscribers(self):
        self.ll_state_subscriber = rospy.Subscriber("ll_state", CarState, callback=self.llStateCallback)
        self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, callback=self.cmdVelCallback)

    def initPublishers(self):
        # TODO: Verify the significance of queue size. Ideally no queue is better
        self.ll_cmd_publisher = rospy.Publisher("ll_cmd", CarControls, queue_size=1)

    def llStateCallback(self, ll_state):
        # self.control_mutex.acquire()

        self.current_velocity = ll_state.velocity  # current_velocity -> km/hr
        self.driving_mode = ll_state.driving_mode
        self.ll_state_ts = time.time()
        # self.control_mutex.release()

    def cmdVelCallback(self, cmd_vel):
        # self.control_mutex.acquire()

        # self.target_velocity = cmd_vel.linear.x  # target_velocity -> km/hr
        self.target_velocity = cmd_vel.linear.x  # target_velocity -> km/hr

        self.target_steering_angle = cmd_vel.angular.z # target_steering_angle -> deg (40, -40) (-ve is right)

        if (cmd_vel.linear.z == 100):
            self.emergency_braking = True
        else:
            self.emergency_braking = False

        self.cmd_vel_ts = time.time()
        # self.control_mutex.release()


    def emptyControlCommand(self):
        ll_cmd = CarControls()
        ll_cmd.throttle = 0
        ll_cmd.brake = 0
        ll_cmd.angle = 0
        ll_cmd.gear = "drive"
        return ll_cmd

    def calculateGear(self):
        # TODO : too risky for now
        # if self.target_velocity >= 0:
        #     return "drive"
        # else:
        #     self.target_velocity -= self.target_velocity
        #     return "reverse"
        return "drive"

    def calculateThrottleBrake(self):
        accln = self.throttle_pid.get_output(self.target_velocity, self.current_velocity)
        if accln > 0:
            throttle_cmd = min(math.ceil(accln), self.throttle_limit)
            brake_cmd = 0
        else:
            throttle_cmd = 0
            brake_cmd = abs(self.brake_pid.get_output(self.target_velocity, self.current_velocity))
            brake_cmd = min(brake_cmd, self.brake_limit)
            # print("BRAKING :: current_vel -", self.current_velocity, " target :", self.target_velocity, ": pedal_val : ", brake_cmd)
            # not using brake
            # brake_cmd = 0 

            if self.target_velocity == 0:
                brake_cmd = self.brake_limit
          
        return int(throttle_cmd), int(brake_cmd)

    def calculateSteer(self):
        # Steering
        steering_angle = 0
        # Clip the magnitude
        steering_angle = min(abs(self.target_steering_angle), 40)
        # Set the direction
        if (self.target_steering_angle < 0): steering_angle = -steering_angle

        return math.ceil(steering_angle)

    def generateControls(self):
        ll_cmd = self.emptyControlCommand()

        # check for data validity
        if (self.cmd_vel_ts == None):
            if not self.log_flags['no_cmd_vel']:
                rospy.logwarn("cmd_vel isn't being published")
            self.log_flags['no_cmd_vel'] = True

            return -1, ll_cmd
        self.log_flags['no_cmd_vel'] = False

        if (self.ll_state_ts == None):
            if not self.log_flags['no_ll_state']:
                rospy.logwarn("ll_state isn't being published")
            self.log_flags['no_ll_state'] = True
            
            return -1, ll_cmd
        self.log_flags['no_ll_state'] = False

        ts_delay = self.cmd_vel_ts - self.ll_state_ts # in seconds
        if (ts_delay > self.ts_delay_threshold):  # ll_state is running behind
            if not self.log_flags["timeout_ll_state"]:
                rospy.logwarn("ll_state in lag by %f ms", ts_delay*100)
            self.log_flags["timeout_ll_state"] = True
            
            return -1, ll_cmd
        self.log_flags["timeout_ll_state"] = False
            
        if ((time.time() - self.cmd_vel_ts) > self.ts_delay_threshold): # cmd_vel not received since sometime
            if not self.log_flags["timeout_cmd_vel"]:        
                rospy.logwarn("cmd_vel is outdated ; received before %f ms", (time.time() - self.cmd_vel_ts)*100)
            self.log_flags["timeout_cmd_vel"] = True

            return -1, ll_cmd
        self.log_flags["timeout_cmd_vel"] = False

        ll_cmd.gear = self.calculateGear()

        ll_cmd.angle = self.calculateSteer()
        ll_cmd.throttle, ll_cmd.brake = self.calculateThrottleBrake()
        
        if self.emergency_braking:
            ll_cmd.throttle, ll_cmd.brake = 0, 50

        ret_val = 0
        return ret_val, ll_cmd

    def runControlsLoop(self):
        while not rospy.is_shutdown():
            tic = time.time()
            if self.driving_mode != "auto":
                if not self.log_flags["not_auto_mode"]:
                    rospy.loginfo("car is in " + self.driving_mode + " mode; not sending ll_cmd")
                self.log_flags["not_auto_mode"] = True
                # We won't send LL Command from here, but ll_interface will send 
                # default commands if it doesn't rcv any ll_command
                continue
            self.log_flags["not_auto_mode"] = False
            
            # self.control_mutex.acquire()
            ret_val, self.generated_ll_cmd = self.generateControls()
            # self.control_mutex.release()
            
            # TODO : Why is this here?
            # There should be a default brake message going
            if ret_val != 0:
                continue
            
            # dt = datetime.datetime.strptime(time, "%Y-%m-%d %H:%M:%S.%f")
            # TODO: Manage print and log statements properly, use rqt_console for viewing logs
            # rospy.loginfo("Publsihing ll_cmd : throttle - " + str(self.generated_ll_cmd.throttle) + " : brake - " + str(self.generated_ll_cmd.brake) + " : angle - " + str(self.generated_ll_cmd.angle))
            self.ll_cmd_publisher.publish(self.generated_ll_cmd)

            toc =time.time()
            tictoc = toc - tic
            time.sleep(0.01 - tictoc) # TODO: This can be faster 
        # self.logs.close()

if __name__ == "__main__":
    rospy.init_node("ll_controller")
    params = rospy.get_param(rospy.get_name())
    #params = rospy.get_param('/')

    ll_controller = LL_controller(params)
    
    ll_controller.runControlsLoop()
    
    # t = Thread(target = ll_controller.runControlsLoop)
    # t.start()
    # rospy.spin()
