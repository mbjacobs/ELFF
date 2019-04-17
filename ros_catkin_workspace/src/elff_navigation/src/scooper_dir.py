#!/usr/bin/env python

# Import necessary modules
import PCA9685 as servo
import time 

RIGHT_SERVO_CHANNEL = 14
LEFT_SERVO_CHANNEL = 15

MIN_PULSE = 200
MAX_PULSE = 700

ARM_DEGREE_INCREMENT = 25

#Global variables to denote the angle of the arms
rightArmDegree = 0 
leftArmDegree = 0

def setup(busnum=None):
	global minLeftArmDegree, maxRightArmDegree, minRightArmDegree, maxLeftArmDegree, home_x, home_y, pwm
	offset_arms = 0
	try:
		for line in open('config'):
			if line[0:8] == 'offset_arms':
				offset_arms = int(line[11:-1])
				#print 'offset_arms =', offset_arm1
	except:
		pass
	minRightArmDegree = MIN_PULSE + offset_arms
	maxRightArmDegree = MAX_PULSE + offset_arms
	minLeftArmDegree = MIN_PULSE + offset_arms
	maxLeftArmDegree = MAX_PULSE + offset_arms
	home_x = (minRightArmDegree + maxRightArmDegree)/2
	home_y = minLeftArmDegree + 80
	
	if busnum == None:
		pwm = servo.PWM() # Initialize the servo controller.
	else:
		pwm = servo.PWM(bus_number=busnum) # Initialize the servo controller.
	pwm.frequency = 60

# Note: sides 'left' and 'right' determined from orientation of looking
# at ELFF from the back.

# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board (the right servo) 
# to make the camera turning towards the positive direction of the x axis.
# ==========================================================================================
def raise_right_arm ():
	global rightArmDegree
	rightArmDegree += ARM_DEGREE_INCREMENT
	        
	if rightArmDegree > maxRightArmDegree:
		rightArmDegree = maxRightArmDegree

	pwm.write(RIGHT_SERVO_CHANNEL, 0, rightArmDegree) 
	
def raise_left_arm ():
	global leftArmDegree
	leftArmDegree -= ARM_DEGREE_INCREMENT

	if leftArmDegree <= minLeftArmDegree:
		leftArmDegree = minLeftArmDegree
		
	pwm.write(LEFT_SERVO_CHANNEL, 0, leftArmDegree) 
	
def raise_arms():
	raise_left_arm()
	raise_right_arm()

# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board (the right servo) 
# to make the camera turning towards the negative direction of the x axis.
# ==========================================================================================
def lower_right_arm ():
	global rightArmDegree
	rightArmDegree -= ARM_DEGREE_INCREMENT

	if rightArmDegree <= minRightArmDegree:
		rightArmDegree = minRightArmDegree
		
	pwm.write(RIGHT_SERVO_CHANNEL, 0, rightArmDegree) 

def lower_left_arm ():
	global leftArmDegree
	leftArmDegree += ARM_DEGREE_INCREMENT
	        
	if leftArmDegree > maxLeftArmDegree:
		leftArmDegree = maxLeftArmDegree

	pwm.write(LEFT_SERVO_CHANNEL, 0, leftArmDegree) 
	
def lower_arms():
	lower_left_arm()
	lower_right_arm()
	
# ==========================================================================================		
# Control the servos connected with channel 14 and 15 at the same time to make the camera 
# move forward.
# ==========================================================================================
def home_x_y():
	global rightArmDegree
	global leftArmDegree
	
	leftArmDegree = home_y 
	rightArmDegree = home_x
	pwm.write(RIGHT_SERVO_CHANNEL, 0, rightArmDegree)
	pwm.write(LEFT_SERVO_CHANNEL, 0, leftArmDegree)

def servo_scoop ():
    for value in range(MIN_PULSE, MAX_PULSE):
        pwm.write(0, 0, value)
        pwm.write(RIGHT_SERVO_CHANNEL, 0, value)
        pwm.write(LEFT_SERVO_CHANNEL, 0, value)
        time.sleep(0.002)

def calibrate(x,y):
	pwm.write(RIGHT_SERVO_CHANNEL, 0, (MAX_PULSE+MIN_PULSE)/2+x)
	pwm.write(LEFT_SERVO_CHANNEL, 0, (MAX_PULSE+MIN_PULSE)/2+y)

def doScoop():
	
	home_x_y()

	for i in range(0, 6):
		lower_arms()
		time.sleep (0.5)
		
	for i in range(0, 6):
		raise_arms()
		time.sleep (0.5)
		
	home_x_y()
	

if __name__ == '__main__':
	setup()
	home_x_y()
