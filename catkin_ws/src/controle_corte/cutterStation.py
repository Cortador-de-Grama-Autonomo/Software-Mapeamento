import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time
from os.path import exists
fuseStep=1 #Valor do passo do fuso em CM
direction= 22 # Pino de direção da GPIO
step = 23 # Pino do passo do motor da GPIO
EN_pin = 24 # Pino de ativação do motor da GPIO
stepMotor = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
GPIO.setup(EN_pin,GPIO.OUT)
pwmBlades = GPIO.PWM(38,100) #Pino de controle do motor das laminas
pwmBlades.start(0)
GPIO.setup(11,GPIO.IN, pull_up_down = GPIO.PUD_DOWN) #Pino conectado ao sensor de tensão


class cutterStation:
    def __init__(self):
        self.emergencyBreak()

    def setHight(self,hight):
        GPIO.output(EN_pin,GPIO.LOW)
        if(exists("currentHight.txt")):
            f = open("currentHight.txt", "r")
            currentHigh=f.read()
        else:
            currentHigh=0
        f.close()
        f = open("currentHight.txt", "a")
        f.write(hight)
        f.close()
        if(currentHigh>hight):
            wise=False
        else:
            wise=True
        stepMotor.motor_go(False, # True=Clockwise, False=Counter-Clockwise
                     "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                     abs(currentHigh-hight)/fuseStep, # number of steps
                     .0005, # step delay [sec]
                     False, # True = print verbose output 
                     .05) # initial delay [sec]
    
    def setSpeed(self,speed):
        pwmBlades.ChangeDutyCycle(speed)
    
    def emergencyBreak(self):
        while True:
            if GPIO.event_detected(11):                  
                pwmBlades.ChangeDutyCycle(0)
            
            time.sleep(0.1)

def main():
    cS = cutterStation()


if __name__ =='__main__':
    main()