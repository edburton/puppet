#!/usr/bin/python

import rospy
import curses
import dynamic_reconfigure.client
from sr_ronex_msgs.msg import PWM, GeneralIOState
from std_msgs.msg import Bool
from math import *

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

active = True;
supressed=False;

def centre_servo(index) :
  return 0.5

def make_waves(index) :
  if supressed :
    return 0.5
  now = rospy.get_rostime()
  time = now.to_sec()-startTime.to_sec()
  minus_one_to_one = sin (time*(1+index/11.0))
  minus_one_to_one/=10;
  zero_to_one = (minus_one_to_one+1.0)/2.0
  return  zero_to_one

def subscriber_cb(msg) :
  global active
  global supressed

  c = stdscr.getch()
  if c == ord(' ') :
    active = not active
    configure_servos(active)
  elif c == ord('z') :
    supressed = not supressed

  if active:
    waves = map(make_waves, range(1, 13))
    set_servo_angles(waves)
  else:
    startTime=rospy.get_rostime()
    

def set_servo_angles(angles_zero_to_one) :
  pwm_message = PWM()
  pwm_message.pwm_period = 64000
  for index, pwm in enumerate(pwms) :
    pwm_message.pwm_on_time_0 =  1600+ angles_zero_to_one[index*2]  * 4800
    pwm_message.pwm_on_time_1 =  1600 + angles_zero_to_one[(index*2)] * 4800
    pwm.publish(pwm_message)
  
def shutdown():
  curses.nocbreak()
  stdscr.keypad(0)
  curses.echo()
  curses.endwin()
  configure_servos(False)

def configure_servos(on):
  client = dynamic_reconfigure.client.Client(ronex_path)
  params = { 'input_mode_0' : not on, 'input_mode_1' : not on, 'input_mode_2' : not on,'input_mode_3' : not on,'input_mode_4' : not on,'input_mode_5' : not on,'input_mode_6' : not on,'input_mode_7' : not on,'input_mode_8' : not on,'input_mode_9' : not on,'input_mode_10' : not on,'input_mode_11' : not on,}
  config = client.update_configuration(params)

if __name__ == "__main__":
  stdscr.clear()
  stdscr.nodelay(1)
  stdscr.addstr("Hello RoNeX\n")
  curses.noecho()
  rospy.init_node("change_ronex_configuration_py")
  rospy.on_shutdown(shutdown)
  configure_servos(True)
  startTime=rospy.get_rostime();
  rospy.Subscriber(ronex_path+"state", GeneralIOState, subscriber_cb)
  rospy.spin()
