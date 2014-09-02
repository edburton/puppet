#!/usr/bin/python
from __future__ import division

import rospy
import curses
import pickle
import dynamic_reconfigure.client
import copy
from sr_ronex_msgs.msg import PWM, GeneralIOState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from math import *
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import plot, ion, show
from matplotlib.mlab import *
from numpy.random import uniform, seed
from matplotlib.mlab import PCA as mlabPCA

from pylab import *

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


#Particle Swarm Optomization variables
w = 0.729844 # Inertia weight to prevent velocities becoming too large
c1 = 1.496180 # Scaling co-efficient on the social component
c2 = 1.496180 # Scaling co-efficient on the cognitive component
dimension = 24 # Size of the problem
particle_swarm_size = 24
particle_positions = np.random.rand(particle_swarm_size,dimension)
for index in range(particle_swarm_size):
    for i in range(dimension):
        particle_positions[index,i]=np.random.uniform(0.4,0.6)
particle_values = [np.nan]*particle_swarm_size
particle_velocities = np.zeros_like(particle_positions)
particle_maxima_positions=np.zeros_like(particle_positions)
particle_maxima_positions[:]=np.nan
particle_maxima_values=[-np.inf]*particle_swarm_size
particle_global_maxima_position=[np.nan]*dimension
particle_global_maxima_value=-np.inf


particle_global_maxima_timeline=[]
particle_global_average_timeline=[]

def func(p) : #Trivial function for testing Particle Swarm Omptomization
    z=0.0
    for n in p:
        for i in range(1,12) :
            z=z+(sin(n*(6.0/i))+(1.0/i))
    return z

frame_counter=0
debug_timer=0;


def subscriber_cb(msg) : #called whenever RoNeX updates (ie: every frame)
    global active, supressed
    global particle_velocities, particle_positions, particle_swarm_size, particle_global_maxima_position, particle_global_maxima_value
    now = rospy.get_rostime()
    time = now.to_sec()-startTime.to_sec()
    global debug_timer
    stdscr.addstr(0,0,str(time-debug_timer)+", ")
    debug_timer=time
    
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
    
    time=time/4.0
    time_int=int(floor(time))
    output_index=time_int%particle_swarm_size
    output_index_2=(output_index+1)%particle_swarm_size
    
       
    
    #output_positions = map(make_waves, range(1, 13)) #get an array of desired servo positions
    if not supressed:
        output_positions = particle_positions[output_index] 
    else:
        output_positions = map(make_waves, range(1, 13))
      
    for index in range(particle_swarm_size): #update velocities
        for i in range(dimension):
            r1 = np.random.uniform()
            r2 = np.random.uniform()
            social=0.0
            cognitive=0.0
            if not np.isnan(particle_global_maxima_position[i]) :
                social = c1 * r1 * (particle_global_maxima_position[i] - particle_positions[index,i])
            if not np.isnan(particle_maxima_positions[index,i]) :
                cognitive = c2 * r2 * (particle_maxima_positions[index,i] - particle_positions[index,i])  
            particle_velocities[index,i] = (w * particle_velocities[index,i]) + social + cognitive          
    
    average=0.0
    
    for index in range(particle_swarm_size): #update cognitive and social maxima
        
        particle_values[index]=func(particle_positions[index,:])
        average=average+particle_values[index]
        if particle_values[index] > particle_global_maxima_value:
            particle_global_maxima_value=particle_values[index]
            for i in range(dimension): 
                particle_global_maxima_position[i]=particle_positions[index,i]
        if particle_values[index] > particle_maxima_values[index]:
            particle_maxima_values[index]=particle_values[index]
            for i in range(dimension):                    
                particle_maxima_positions[index,i]=particle_positions[index,i]
        
    average= average/particle_swarm_size
    particle_global_average_timeline.append(average+np.random.uniform(0.2,0.4))
    particle_global_maxima_timeline.append(particle_global_maxima_value+np.random.uniform(0.4,0.6))
    
    #particle_positions=np.add(particle_positions,particle_velocities)   #add velocities to positions

    #INSERT INTERESTING BIT HERE    
    
    #------------------------
        
    if active:
        set_servo_angles(output_positions)
    #else: 
    #    startTime=rospy.get_rostime()


def get_servo_positions(msg) : #get analogue value from an individual servo
    analogue_inputs = []
    for index in range(12): #fill an array with the analogue servo inputs
        analogue_inputs.append(msg.analogue[index]) # range 0 -- 3690
    return analogue_inputs 


def make_waves(index) : #generates out-of-sync sine waves for each servo
    if supressed :
        return 0.5
    now = rospy.get_rostime()
    time = now.to_sec()-startTime.to_sec()
    minus_one_to_one = sin (time*(1+index/12.0))
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
    plt.show()
    plt.ion()
    

fig = plt.figure()
ax1 = fig.add_subplot(211)
average_graph, = ax1.plot([], [],label='average')
plt.legend(loc='best')

ax2 = fig.add_subplot(212)
max_graph, = ax2.plot([], [],label='max')
plt.legend(loc='best')
    
def init():
    average_graph.set_data([],[])
    max_graph.set_data([],[])
    return ax1, ax2, 

# animation function.  This is called sequentially
def animate(i):
    global particle_global_average_timeline
    p=list(particle_global_average_timeline)
    n = len(p)
    x = np.arange(0,n)
    ax1.set_xlim(0,n)
    ax1.set_ylim(min(p),max(p))
    average_graph.set_data(x, p)
    
    global particle_global_maxima_timeline
    p=list(particle_global_maxima_timeline)
    n = len(p)
    x = np.arange(0,n)
    ax2.set_xlim(0,n)
    ax2.set_ylim(min(p),max(p))
    max_graph.set_data(x, p)
    
    return ax1, ax2,



if __name__ == "__main__": #setup
    #setup_visualisation()
    rospy.init_node("change_ronex_configuration_py")
    configure_servos(active)
    stdscr.clear()
    stdscr.nodelay(1)
    stdscr.addstr("Hello RoNeX\n")
    curses.noecho() #make the terminal accept individual key-presses and not echo them to the screen
    rospy.on_shutdown(shutdown)
    startTime=rospy.get_rostime();
    rospy.Subscriber(ronex_path+"state", GeneralIOState, subscriber_cb)
    
    anim = animation.FuncAnimation(fig, animate, init_func=init, interval=250, blit=False)
    plt.show() 
    rospy.spin()
    

# initialization function: plot the background of each frame
