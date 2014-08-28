#!/usr/bin/python

import rospy
import curses
import pickle
import dynamic_reconfigure.client
from sr_ronex_msgs.msg import PWM, GeneralIOState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from math import *
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import plot, ion, show
from numpy.random import uniform, seed
from matplotlib.mlab import griddata

ronex_id = "1403017360"
ronex_path = "/ronex/general_io/" + ronex_id + "/"

startTime=-1;

pub0 = rospy.Publisher(ronex_path+"command/pwm/0", PWM)
pub1 = rospy.Publisher(ronex_path+"command/pwm/1", PWM)
pub2 = rospy.Publisher(ronex_path+"command/pwm/2", PWM)
pub3 = rospy.Publisher(ronex_path+"command/pwm/3", PWM)
pub4 = rospy.Publisher(ronex_path+"command/pwm/4", PWM)
pub5 = rospy.Publisher(ronex_path+"command/pwm/5", PWM)

pwms = [pub0,pub1,pub2,pub3,pub4,pub5]

stdscr = curses.initscr()

active = False 
supressed=True

analogue_input_queue = deque([],4)

pub_output = rospy.Publisher('puppet', Float32)

particle_population_size = 24
test_xs = np.random.rand(particle_population_size)
test_ys = np.random.rand(particle_population_size)

def subscriber_cb(msg) : #called whenever RoNeX updates (ie: every frame)
    global active
    global supressed
    
    c = stdscr.getch() #get keypresses from console 
    if c != curses.ERR:
        if c == ord(' ') : #space bar toggles all servos on/off
            active = not active
            configure_servos(active)
        elif c == ord('z') : #z toggles wave behaviour on/off
            supressed = not supressed
    
    #------------------------
    
    analogue_inputs = get_servo_positions(msg)

    analogue_input_queue.append(analogue_inputs)
    
    if len(analogue_input_queue)>3:
        a1 = []
        a2 = []
        for n in range(0, 12):
            a1.append((analogue_input_queue[0][n]+analogue_input_queue[1][n])/2)
            a2.append((analogue_input_queue[2][n]+analogue_input_queue[3][n])/2)
        d = 0.0
        for n in range(0, 12):
            d=d+(a1[n]-a2[n])**2
        d = sqrt(d)
        pub_output.publish(d)
        #rospy.loginfo("analogue_input_queue.length = %d", len(analogue_input_queue))
    
    
    
        output_positions = map(make_waves, range(1, 13)) #get an array of desired servo positions
    
        for index in range(0, particle_population_size) :
            test_xs[index]=test_xs[index]+(np.random.normal()/100)
            test_ys[index]=test_ys[index]+(np.random.normal()/100)
        
        plt.clf()
        plt.scatter(x=test_xs, y=test_ys)
    
        plt.draw()
    #INSERT INTERESTING BIT HERE    
    
    #------------------------
        
    if active:
        set_servo_angles(output_positions)
    #else:
    #    startTime=rospy.get_rostime()


def get_servo_positions(msg) : #get analogue value from an individual servo
    analogue_inputs = []
    for index in range(0, 12): #fill an array with the analogue servo inputs
        analogue_inputs.append(msg.analogue[index]) # range 0 -- 3690
    return analogue_inputs 


def make_waves(index) : #generates out-of-sync sine waves for each servo
    if supressed :
        return 0.5
    now = rospy.get_rostime()
    time = now.to_sec()-startTime.to_sec()
    minus_one_to_one = sin (time*(1+index/6.0))
    minus_one_to_one/=10;
    zero_to_one = (minus_one_to_one+1.0)/2.0
    return  zero_to_one


def angle_zero_to_one_to_pwm(angle_zero_to_one) :
    return  1600+ angle_zero_to_one  * 4800


def set_servo_angle(angle_zero_to_one,servo_index) : #set an individual servo position
    pwm_message = PWM()
    pwm_message.pwm_period = 64000
    pwm_module=servo_index/2; #RoNeX has six PWM modules each driving two servos
    if servo_index % 2 == 0 :
        pwm_message.pwm_on_time_0 =  angle_zero_to_one_to_pwm(angle_zero_to_one)
    else :
        pwm_message.pwm_on_time_1 =  angle_zero_to_one_to_pwm(angle_zero_to_one)
    

def set_servo_angles(angles_zero_to_one) : #set all the servo from a list of positions
    pwm_message = PWM()
    pwm_message.pwm_period = 64000
    for index, pwm in enumerate(pwms) : #RoNeX has six PWM modules each driving two servos 
        pwm_message.pwm_on_time_0 =  angle_zero_to_one_to_pwm(angles_zero_to_one[index*2])
        pwm_message.pwm_on_time_1 =  angle_zero_to_one_to_pwm(angles_zero_to_one[(index*2)+1])
        pwm.publish(pwm_message)

  
def shutdown(): #put the console back to normal and turn the servos off
    configure_servos(False)
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()
    plt.close('all')
    

def configure_servos(on): #turn all servos on or off
    client = dynamic_reconfigure.client.Client(ronex_path)
    params = { 'input_mode_0' : not on, 'input_mode_1' : not on, 'input_mode_2' : not on,'input_mode_3' : not on,'input_mode_4' : not on,'input_mode_5' : not on,'input_mode_6' : not on,'input_mode_7' : not on,'input_mode_8' : not on,'input_mode_9' : not on,'input_mode_10' : not on,'input_mode_11' : not on,}
    config = client.update_configuration(params)

def setup_visualisation():
    plt.ion()
    plt.show()
    
if __name__ == "__main__": #setup
    setup_visualisation()
    
    rospy.init_node("change_ronex_configuration_py")
    rospy.on_shutdown(shutdown)
    configure_servos(active)
    startTime=rospy.get_rostime();
    rospy.Subscriber(ronex_path+"state", GeneralIOState, subscriber_cb)

    stdscr.clear()
    stdscr.nodelay(1)
    stdscr.addstr("Hello RoNeX\n")
    curses.noecho() #make the terminal accept individual key-presses and not echo them to the screen
        
    
    rospy.spin()
