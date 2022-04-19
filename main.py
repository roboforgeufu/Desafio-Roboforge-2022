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
motorD = Motor(Port.D)
motorA = Motor(Port.A)
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)
sensorc3 = ColorSensor(Port.S3)
cronometro = StopWatch()
#robo = DriveBase(test_motorB,test_motorC,8.5,24)

def segue_linha(vel):
    valorLum = 35 #medir na linha toda vez
    Kp = 3
    Ki = 0
    Kd = 3
    erro = 0
    integral = 0
    while True: #!!!TESTAR!!!
        erro0 = erro
        erro = valorLum - sensorc3.reflection()  
        derivada = erro - erro0
        integral = integral + erro
        valorP = erro*Kp
        valorI = integral*Ki
        valorD = derivada*Kd
        valorPID = valorP + valorI + valorD
        if(valorPID>=0):
            motorC.run(vel+valorPID)
            motorB.run(vel+valorPID*0.5)
        if(valorPID<0):
            motorC.run(vel+(-valorPID*0.5))
            motorB.run(vel+(-valorPID))

def alinhar():
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        motorC.run(50)
        motorB.run(50) #identifica a linha preta
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    while(sensorc2.color()!=Color.BLACK):
        motorB.run(50)
    motorB.hold()
    while(sensorc2.color()!=Color.WHITE): #alinha o motor B
        motorB.run(50)
    motorB.reset_angle(0)
    ev3.speaker.beep()
    motorB.hold()
    motorB.run_target(50,-50,then=Stop.HOLD)
    while(sensorc2.color()!=Color.WHITE):
        motorB.run(-50)
    ev3.speaker.beep()
    motorB.hold()
    angB = motorB.angle()
    motorB.reset_angle(0)
    motorB.run_target(50,-angB/2,then=Stop.HOLD)

    while(sensorc3.color()!=Color.BLACK):
        motorC.run(50)
    motorC.hold()
    while(sensorc3.color()!=Color.WHITE): #alinha o motor C
        motorC.run(50)
    motorC.reset_angle(0)
    ev3.speaker.beep()
    motorC.hold()
    motorC.run_target(50,-50,then=Stop.HOLD)
    while(sensorc3.color()!=Color.WHITE):
        motorC.run(-50)
    ev3.speaker.beep()
    motorC.hold()
    angC = motorC.angle()
    motorC.reset_angle(0)
    motorC.run_target(50,-angB/2,then=Stop.HOLD)
        
def deixa_bloco():
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        motorC.run(50)
        motorB.run(50)
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    while(sensorc2.color()!=Color.WHITE and sensorc3.color()!=Color.WHITE):
        motorC.run(50)
        motorB.run(50)
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    cronometro.reset()
    while(cronometro.time()<3000):
        motorA.dc(-40)

def identifica_base():
    while True:
        segue_linha(100)
        if(sensorc1.color()==Color.BLUE):
            break

def main():
    while True:
        if(sensorc1.color()==Color.BLUE):
            ev3.speaker.beep()

main()
