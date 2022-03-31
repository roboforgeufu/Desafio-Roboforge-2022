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
#cronometro = StopWatch()
#robo = DriveBase(test_motorB,test_motorC,8.5,24)

ev3.speaker.beep()

mediaV=300 #velocidade media no duty cycle referido
n=0
media=[]
motorA.reset_angle(0)
while True:
    motorA.dc(40)
    velA=motorA.speed()
    if(velA>260):
        media.append(velA) #joga os valores válidos (sem ruído) em uma lista
        if(len(media)==5): #quando a lista atinge 5 valores calcula a média entre eles
            for i in range(5): 
                mediaV = mediaV + media[i]
            mediaV = mediaV/6
            media.clear()
            n=n+1
    if(n>10 and mediaV<280): #descarta os 50 primeiros valores válidos (aceleração incial) antes de comparar
        break
angGarra = motorA.angle()
if(angGarra<470):
    ev3.speaker.beep(100)
else:
    ev3.speaker.beep(500)
