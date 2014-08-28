#!/usr/bin/python

import rospy
import dynamic_reconfigure.client
from sr_ronex_msgs.msg import PWM, GeneralIOState
from std_msgs.msg import Bool

ronex_id = "1403017360"

previousOntime=-1;

pub0 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/0", PWM)
pub1 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/1", PWM)
pub2 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/2", PWM)
pub3 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/3", PWM)
pub4 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/4", PWM)
pub5 = rospy.Publisher("/ronex/general_io/"+ronex_id+"/command/pwm/5", PWM)

def subscriber_cb(msg) :

  analog_in = float(msg.analogue[0]) # range 0 -- 3670
  analog_in /= 1570	             # range 0 - 1
  ontime = analog_in + .5	     # on time in ms

  period = 65000                    # in clock ticks

  clock_speed = 64e6/20            #64MHz clock, clock divider of 20

  clock_tick = (1/clock_speed) * 1000 # in mseconds

  ontime /= clock_tick

  global previousOntime
  if abs(ontime-previousOntime)>10:
    print ontime
    previousOntime=ontime

  pwm_message = PWM()
  pwm_message.pwm_period = period
  pwm_message.pwm_on_time_0 = ontime
  pwm_message.pwm_on_time_1 = ontime
	 
  
  pub0.publish(pwm_message)
  pub1.publish(pwm_message)
  pub2.publish(pwm_message)
  pub3.publish(pwm_message)
  pub4.publish(pwm_message)
  pub5.publish(pwm_message)


class ChangeRonexConfigurationExample(object):

    def __init__(self):
        ronex_path = "/ronex/general_io/" + ronex_id + "/"
        self.configure_ronex(ronex_path)

    def configure_ronex(self, path):
        client = dynamic_reconfigure.client.Client(path)
        params = { 'input_mode_0' : False, 'input_mode_1' : False, 'input_mode_2' : False,'input_mode_3' : False,'input_mode_4' : False,'input_mode_5' : False,'input_mode_6' : False,'input_mode_7' : False,'input_mode_8' : False,'input_mode_9' : False,'input_mode_10' : False,'input_mode_11' : False,}
        config = client.update_configuration(params)

if __name__ == "__main__":
	
    rospy.init_node("change_ronex_configuration_py")
    ChangeRonexConfigurationExample()
    rospy.Subscriber("/ronex/general_io/"+ronex_id+"/state", GeneralIOState, subscriber_cb)
    rospy.spin()
