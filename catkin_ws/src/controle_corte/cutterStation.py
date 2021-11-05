import time
from os.path import exists
if __name__ =='__main__':
    DEBUG = True
else:
    DEBUG = False
if(DEBUG):
    fuseStep=0.8 #Valor do passo do fuso em CM
    direction= 22 # Pino de direcao da GPIO
    step = 23 # Pino do passo do motor da GPIO
    EN_pin = 24 # Pino de ativacao do motor da GPIO
else:
    import RPi.GPIO as GPIO
    from RpiMotorLib import RpiMotorLib
    fuseStep=0.8 #Valor do passo do fuso em CM
    direction= 22 # Pino de direcao da GPIO
    step = 23 # Pino do passo do motor da GPIO
    EN_pin = 24 # Pino de ativacao do motor da GPIO
    stepMotor = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
    GPIO.setup(EN_pin,GPIO.OUT)
    pwmBlades = GPIO.PWM(38,100) #Pino de controle do motor das laminas
    pwmBlades.start(0)
    GPIO.setup(11,GPIO.IN, pull_up_down = GPIO.PUD_DOWN) #Pino conectado ao sensor de tensao

class cutterStation:
    def __init__(self):
        if(DEBUG):
            print("Debug ativado")
        else:
            self.emergencyBreak()

    def setHight(self,hight):
        if (DEBUG):
            print('Altura setada em: %d cm' % (hight))
        else:
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
            stepMotor.motor_go(wise, # True=Clockwise, False=Counter-Clockwise
                        "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                        abs(currentHigh-hight)/fuseStep, # number of steps
                        .0005, # step delay [sec]
                        False, # True = print verbose output 
                        .05) # initial delay [sec]
    
    def setSpeed(self,speed):
        if(speed==1):
            speed_p=33
        elif(speed==2):
            speed_p=66
        else:
            speed_p=100
        if(DEBUG):
            print("Velocidade selecionada e %d que corresponde a %d porcento da velocidade do motor." %(speed,speed_p))
        else:
            pwmBlades.ChangeDutyCycle(speed_p/100)
    
    def emergencyBreak(self):
        if(DEBUG):
            print("Emergencia ativada")
        else:
            while True:
                if GPIO.event_detected(11):                  
                    pwmBlades.ChangeDutyCycle(0)
                
                time.sleep(0.1)

def main():
    cS = cutterStation()
    cS.setHight(1)
    cS.setSpeed(1)
    cS.emergencyBreak()

if __name__ =='__main__':
    main()