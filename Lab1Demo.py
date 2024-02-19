from Motor import *            
PWM=Motor() 

from Ultrasonic import *
ultrasonic=Ultrasonic() 

from servo import *
pwm_servo=Servo()

import time
from random import random


def go():
	PWM.setMotorModel(1000, 1000, 1000, 1000)  # Forward
	
def manuver():
	PWM.setMotorModel(0, 0, 0, 0)  # Stop
	
	PWM.setMotorModel(-1000, -1000, -1000, -1000)  # Backward
	time.sleep(0.5)
	
	if random() > 0.5:
		PWM.setMotorModel(-500, -500, 2000, 2000)  # Left
		time.sleep(0.5)
	else:
		PWM.setMotorModel(2000, 2000, -500, -500)  # Right
		time.sleep(0.5)
	
	PWM.setMotorModel(0, 0, 0, 0)  # Stop
	time.sleep(3)
	

if __name__ == '__main__':
	try:
		print ('Program is starting ... ')
		pwm_servo.setServoPwm('0',90)
		pwm_servo.setServoPwm('1',110)
		while True:
			distance = ultrasonic.get_distance()
			if distance < 10:
				manuver()
			else:
				go()	
	except KeyboardInterrupt:
		PWM.setMotorModel(0,0,0,0)
		print ("\nEnd of program")
