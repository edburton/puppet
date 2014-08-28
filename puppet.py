#!/usr/bin/python

import rospy
import dynamic_reconfigure.client
from sr_ronex_msgs.msg import PWM, GeneralIOState
from std_msgs.msg import Bool
from math import *

ronex_id = "1403017360"

previousOntime=-1;

pub0 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/0", PWM)
pub1 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/1", PWM)
pub2 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/2", PWM)
pub3 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/3", PWM)
pub4 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/4", PWM)
pub5 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/5", PWM)

pwms = [pub0,pub1,pub2,pub3,pub4,pub5]

def make_waves(index) :
  now = rospy.get_rostime()
  time = now.to_sec()
  return (cos(((time*(1+index/20.0))+(index*3))/2)/12.0)+1

def subscriber_cb(msg) :
  analog_in = float(msg.analogue[0]) # range 0 -- 3690
  print analog_in  
  analog_in /= 3690	             # range 0 - 1
  
  waves = map(make_waves, range(1, 13))

  period = 65000                    # in clock ticks

  clock_speed = 64e6/20            #64MHz clock, clock divider of 20

  clock_tick = (1/clock_speed) * 1000 # in mseconds

  pwm_message = PWM()
  pwm_message.pwm_period = period

  for index, pwm in enumerate(pwms) :
    pwm_message.pwm_on_time_0 = ( waves[index*2] + .5 ) / clock_tick
    pwm_message.pwm_on_time_1 = ( waves[(index*2)+1] + .5 ) / clock_tick
    pwm.publish(pwm_message)


class ChangeRonexConfiguration(object):

  def __init__(self):
    ronex_path = "/ronex/general_io/" + ronex_id + "/"
    self.configure_ronex(ronex_path)

  def configure_ronex(self, path):
    client = dynamic_reconfigure.client.Client(path)
    params = { 'input_mode_0' : False, 'input_mode_1' : False, 'input_mode_2' : False,'input_mode_3' : False,'input_mode_4' : False,'input_mode_5' : False,'input_mode_6' : False,'input_mode_7' : False,'input_mode_8' : False,'input_mode_9' : False,'input_mode_10' : False,'input_mode_11' : False,}
    config = client.update_configuration(params)

if __name__ == "__main__":
	
  rospy.init_node("change_ronex_configuration_py")
  ChangeRonexConfiguration()

  rospy.Subscriber("/ronex/general_io/"+ronex_id+"/state", GeneralIOState, subscriber_cb)
  rospy.spin()
