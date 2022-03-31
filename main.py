#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch
# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motorB = Motor(Port.B)
motorC = Motor(Port.C)
motorA = Motor(Port.A)
#sensorc1 = ColorSensor(Port.S1)
#sensorc2 = ColorSensor(Port.S2)
sensorc3 = ColorSensor(Port.S3)
cronometro = StopWatch()
#robo = DriveBase(test_motorB,test_motorC,8.5,24)

def andar_igual(vel,time):
    cronometro.reset()
    while(cronometro.time()<time):
        motorB.run(vel)
        vB = motorB.speed()
        motorC.run(vB)
    motorB.hold()
    motorC.hold()

def andar_bloco():
    while(sensorc3.reflection()<10):
        motorB.run(50)
        motorC.run(50)
    andar_igual(-50,1500)

def identifica_bloco():
    mediaV=280 #velocidade media no duty cycle referido
    n=0
    media=[]
    motorA.reset_angle(0)
    while True:
        motorA.dc(40)
        velA=motorA.speed()
        if(velA>250):
            media.append(velA) #joga os valores válidos (sem ruído) em uma lista
            if(len(media)==5): #quando a lista atinge 5 valores calcula a média entre eles
                for i in range(5): 
                    mediaV = mediaV + media[i]
                mediaV = mediaV/6
                media.clear()
                n=n+1
        if(n>10 and mediaV<255): #descarta os 50 primeiros valores válidos (aceleração incial) antes de comparar
            print(mediaV)
            break
    angGarra = motorA.angle()
    if(angGarra<470):
        return 1 #bloco grande
    else:
        return 0 #bloco pequeno

def main():
    andar_bloco()
    identifica_bloco()

main()
