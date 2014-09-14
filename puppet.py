#!/usr/bin/python
from __future__ import division

import rospy
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

pub0 = rospy.Publisher(ronex_path+"command/pwm/0", PWM)
pub1 = rospy.Publisher(ronex_path+"command/pwm/1", PWM)
pub2 = rospy.Publisher(ronex_path+"command/pwm/2", PWM)
pub3 = rospy.Publisher(ronex_path+"command/pwm/3", PWM)
pub4 = rospy.Publisher(ronex_path+"command/pwm/4", PWM)
pub5 = rospy.Publisher(ronex_path+"command/pwm/5", PWM)

pwms = [pub0,pub1,pub2,pub3,pub4,pub5]

analogue_input_queue = deque([],4) 

pub_output = rospy.Publisher('puppet', Float32)

#Particle Swarm Optomization variables
w = 0.729844 # Inertia weight to prevent velocities becoming too large
c1 = 1.496180 # Scaling co-efficient on the social component
c2 = 1.496180 # Scaling co-efficient on the cognitive component

servos = 12

dimension = servos*2 # Size of the problem
particle_swarm_size = 6
particle_positions = np.random.rand(particle_swarm_size,dimension)
for index in range(particle_swarm_size):
    for i in range(dimension):
        particle_positions[index,i]=np.random.uniform(0.4,0.6)
particle_values = [np.NINF]*particle_swarm_size
particle_values_for_average = [0.0]*particle_swarm_size
particle_velocities = np.zeros_like(particle_positions)
particle_maxima_positions=np.zeros_like(particle_positions)
particle_maxima_positions[:]=np.nan
particle_maxima_values=[np.NINF]*particle_swarm_size
particle_global_maxima_position=[np.nan]*dimension
particle_global_maxima_value=np.NINF

particle_global_maxima_timeline=[]
particle_global_average_timeline=[]

gesture_duration=1.0
relaxation_pause_duration=0.5
restart_pause_duration=0.25
posture_portion=0.25

particle_of_interest=np.nan
next_particle_of_interest=0;
particle_of_interest_start_time=0.0
particle_modes=['null','first_gesture','relaxation_pause','restart_pause','second_gesture','end_pause']
particle_of_interest_mode=0

inputs_at_end_of_gesture=np.nan
inputs_at_rest=np.nan
inputs_at_repeat_posture=np.nan

output_positions=[0.5]*servos

def func(p) : #Trivial function for testing Particle Swarm Omptomization
    z=0.0
    for n in p:
        for i in range(1,servos) :
            z=z+(sin(n*(6.0/i))+(1.0/i))
    return z

frame_counter=0
debug_timer=0;

def accelerate(t) :
    t=(t-posture_portion)*(0.5/(0.5-posture_portion))
    t=np.clip(t,0,1)
    return (1-cos(t*np.pi))/2

def subscriber_cb(msg) : #called whenever RoNeX updates (ie: every frame)
    global gesture_duration, posture_duration, pause_duration, output_positions, particle_values_for_average
    global inputs_at_end_of_gesture, inputs_at_rest, inputs_at_repeat_posture
    global particle_of_interest, next_particle_of_interest, particle_of_interest_start_time,particle_modes,particle_of_interest_mode
    global particle_velocities, particle_positions, particle_swarm_size, particle_global_maxima_position, particle_global_maxima_value
    now = rospy.get_rostime()
    time = now.to_sec()
    global debug_timer
    #print(str(time-debug_timer))
    debug_timer=time
    
    #------------------------
    
    analogue_inputs = get_servo_positions(msg)
    
    analogue_input_queue.append(analogue_inputs)
    
    
    if np.isnan(particle_of_interest):
        particle_of_interest=next_particle_of_interest
        particle_of_interest_start_time=time
        particle_of_interest_mode=1
        configure_servos(True)
        print('particle_of_interest = '+str(particle_of_interest))
    
    if particle_of_interest_mode==1:
        t=accelerate((time-particle_of_interest_start_time)/gesture_duration)
        for i in range(servos):
            output_positions[i]=(particle_positions[particle_of_interest,i]*(1-t))+(particle_positions[particle_of_interest,servos+i]*t)
    
    if particle_of_interest_mode==1:
        if (time-particle_of_interest_start_time)>gesture_duration:
            inputs_at_end_of_gesture=get_smoothed_servo_positions()
            configure_servos(False)
            particle_of_interest_mode=2
            
    if particle_of_interest_mode==2:
        if (time-particle_of_interest_start_time)>gesture_duration+relaxation_pause_duration:
            particle_of_interest_mode=3  
            inputs_at_rest=get_smoothed_servo_positions()   
            
    if particle_of_interest_mode==3:
        if (time-particle_of_interest_start_time)>gesture_duration+relaxation_pause_duration+restart_pause_duration:
            particle_of_interest_mode=4  
            configure_servos(True)
                
    if particle_of_interest_mode==4 or particle_of_interest_mode==5:
        t=accelerate((time-(particle_of_interest_start_time+gesture_duration+relaxation_pause_duration+restart_pause_duration))/gesture_duration)
        for i in range(servos):
            output_positions[i]=(particle_positions[particle_of_interest,i]*(1-t))+(particle_positions[particle_of_interest,servos+i]*t)
    
    if (particle_of_interest_mode==4 and (time-particle_of_interest_start_time)>gesture_duration+relaxation_pause_duration+restart_pause_duration+(gesture_duration*posture_portion)):
        inputs_at_repeat_posture=get_smoothed_servo_positions()
        particle_of_interest_mode=5   
        
    if particle_of_interest_mode==5:
        if (time-particle_of_interest_start_time)>((gesture_duration*2)+relaxation_pause_duration+restart_pause_duration):
            configure_servos(False)
            particle_of_interest_mode=6  
    
    new_fitness_for_particle=np.nan
    
    if particle_of_interest_mode==6:
        if (time-particle_of_interest_start_time)>((gesture_duration*2)+((relaxation_pause_duration+restart_pause_duration)*2)):
            d1=np.linalg.norm(inputs_at_end_of_gesture-inputs_at_rest)
            d2=np.linalg.norm(inputs_at_repeat_posture-inputs_at_rest)
            fitness=d1-d2
            if fitness<0:
                fitness=-sqrt(fitness/-1000)  #crush exceptionally negative fitnesses caused by brown-out
            print ("finishing="+str(d1)+", repeating="+str(d2)+", fitness="+str(fitness))
            particle_values[particle_of_interest]=fitness
            particle_values_for_average[particle_of_interest]=fitness
            new_fitness_for_particle=particle_of_interest
            next_particle_of_interest=(particle_of_interest+1)%particle_swarm_size
            particle_of_interest=np.nan
            particle_of_interest_start_time=np.nan
            particle_of_interest_mode=0
            inputs_at_end_of_gesture=np.nan
            inputs_at_rest=np.nan
            inputs_at_repeat_posture=np.nan

       
      
    for index in range(particle_swarm_size): #update velocities
        if index==new_fitness_for_particle:
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
        average=average+particle_values_for_average[index]
        #particle_values[index]=func(particle_positions[index,:])
        if not np.isneginf(particle_values[index]) :
            average=average+particle_values[index]
            if particle_values[index] > particle_global_maxima_value:
                particle_global_maxima_value=particle_values[index]
                particle_global_maxima_position=np.copy(particle_positions[index])
            if particle_values[index] > particle_maxima_values[index]:
                particle_maxima_values[index]=particle_values[index]
                particle_maxima_positions[index]=np.copy(particle_positions[index])
    
    average= average/particle_swarm_size

    
    if not np.isnan(new_fitness_for_particle):
        particle_global_average_timeline.append(average)
        particle_global_maxima_timeline.append(particle_global_maxima_value)
        particle_positions[new_fitness_for_particle]=np.add(particle_positions[new_fitness_for_particle],particle_velocities[new_fitness_for_particle])   #add velocities to positions
        particle_values[new_fitness_for_particle]=np.NINF
    #INSERT INTERESTING BIT HERE    
    
    #------------------------
        
    if not np.isnan(particle_of_interest):
        set_servo_angles(output_positions)
    #else: 
    #    startTime=rospy.get_rostime()


def get_servo_positions(msg) : #get analogue value from an individual servo
    analogue_inputs_ = []
    for index in range(servos): #fill an array with the analogue servo inputs
        analogue_inputs_.append(msg.analogue[index]) # range 0 -- 3690
    return analogue_inputs_ 


def get_smoothed_servo_positions() :
    analogue_inputs_ = np.array([0.0]*servos)
    q=len(analogue_input_queue)
    for index in range(servos):
        for n in range(q):
            analogue_inputs_[index]=analogue_inputs_[index]+analogue_input_queue[n][index]
        if q>0:
            analogue_inputs_[index]=analogue_inputs_[index]/q
    return analogue_inputs_

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
    centre_servos()
    plt.close('all')
    configure_servos(False)


def centre_servos():
    pwm=np.empty(servos); pwm.fill(0.5) 
    set_servo_angles(pwm)   
    configure_servos(True)
    set_servo_angles(pwm)


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
    p=np.array(particle_global_average_timeline)
    n = len(p)
    if n>0:
        x = np.arange(0,n)
        ax1.set_xlim(0,n)
        ax1.set_ylim(min(p),max(p))
        average_graph.set_data(x, p)
    
    global particle_global_maxima_timeline
    p=np.array(particle_global_maxima_timeline)
    n = len(p)
    if n>0:
        x = np.arange(0,n)
        ax2.set_xlim(0,n)
        ax2.set_ylim(min(p),max(p))
        max_graph.set_data(x, p)
    
    return ax1, ax2,



if __name__ == "__main__": #setup
    #setup_visualisation()
    rospy.init_node("change_ronex_configuration_py")
    centre_servos()
    
    rospy.on_shutdown(shutdown)
    rospy.Subscriber(ronex_path+"state", GeneralIOState, subscriber_cb)
    
    anim = animation.FuncAnimation(fig, animate, init_func=init, interval=250, blit=False)
    plt.show() 
    rospy.spin()
    

