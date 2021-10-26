
import RPi.GPIO as gpio
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)

gpio.setup(23, gpio.OUT) #driver locomoção motor direito (lógico)
gpio.setup(24, gpio.OUT) #driver locomoção motor direito (PWM)
gpio.setup(9, gpio.OUT) #driver locomoção motor esquerdo (lógico)
gpio.setup(18, gpio.OUT) #driver locomoção motor esquerdo (PWM)

class ConverVelocities():
	def __init__(self):
		self.twist_vels = Twist()
		self.vr = Float64()
		self.vl = Float64()
		self.rate = rospy.Rate(1)
		self.vr_pwm = GPIO.PWM(24, 1000)
		self.vl_pwm = GPIO.PWM(18, 1000)
		self.vr_pwm.start(0)
		self.vl_pwm.start(0)
		self.sub = rospy.Subscriber('/twist_vels', Twist, self.leitura_callback)

	def leitura_callback(self, msg):
        	self.twist_vels = msg
        
        def pub_wheel_vels(self):
		while not rospy.is_shutdown():
		    self.convert_velocities()
		    self.rate.sleep()
		    
 	def convert_velocities(self):
		# Distância entre as rodas (m) = 0.8
		# Raio das Rodas (m) = 0.1016
		# Rotação máxima (rad/s) = 9.842519685
		state_description = ''
		self.vr = ((2*self.twist_vels.linear.x) + (self.twist_vels.angular.z*0.8))/(2*0.1016)
		self.vl = ((2*self.twist_vels.linear.x) - (self.twist_vels.angular.z*0.8))/(2*0.1016)
		self.vr_pwm.ChangeDutyCycle((self.vr/9.842519685)*100)
        	self.vl_pwm.ChangeDutyCycle((self.vl/9.842519685)*100)
        	if self.vr > self.vl:
			state_description = 'caso 1 - virar a esquerda'
			gpio.output(23,True)
			gpio.output(9, False)
		elif self.vr < self.vl:
			state_description = 'caso 2 - Virar a direita'
			gpio.output(23,False)
			gpio.output(9, True)
		else:
			state_description = 'caso 3 - Andar para frente'
			gpio.output(23,True)
			gpio.output(9, True)
			
if __name__ == '__main__':
    rospy.init_node('convert_vels_node', anonymous=True)
    cv = ConverVelocities()
    cv.pub_wheel_vels()  
        
        
        
        
        
        
        
        
