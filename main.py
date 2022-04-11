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
sensorc4 = ColorSensor(Port.S4)
cronometro = StopWatch()
#robo = DriveBase(test_motorB,test_motorC,8.5,24)

def andar_igual_tempo(vel,time):
    cronometro.reset()
    ajuste = 0
    while(cronometro.time()<time):
        if(ajuste<1):
            ajuste = ajuste + 0.002
        motorC.run(vel*ajuste)
        vC = motorC.speed()
        motorB.run(vC) #joga o valor de vel lido para o segundo motor
    motorB.hold()
    motorC.hold()

def andar_igual(vel):
    motorB.run(vel)
    vB = motorB.speed()
    motorC.run(vB) #joga o valor de vel lido para o segundo motor

def andar_bloco():
    while(sensorc4.reflection()<10): #vai para frente até o bloco tampar o sensor
        motorB.run(50)
        vB = motorB.speed()
        motorC.run(vB)
    andar_igual_tempo(-50,400)
    cor = sensorc3.color() #le a cor do bloco
    andar_igual_tempo(-50,2600)
    return cor

def identifica_bloco():
    mediaV=330 #velocidade media no duty cycle referido (340 bateria cheia)
    n=0
    media=[]
    motorA.reset_angle(0)
    while True:
        motorA.dc(40)
        velA=motorA.speed()
        #print(velA)
        if(velA>310): # 320 bateria cheia
            media.append(velA) #joga os valores válidos (sem ruído) em uma lista
            if(len(media)==5): #quando a lista atinge 5 valores calcula a média entre eles
                for i in range(5): 
                    mediaV = mediaV + media[i]
                mediaV = mediaV/6
                media.clear()
                n=n+1
        if(n>10 and mediaV<315): #descarta os 50 primeiros valores válidos (aceleração incial) antes de comparar

            break
    angGarra = motorA.angle()
    if(angGarra<530):
        return 1,angGarra #bloco grande
    else:
        return 0,angGarra #bloco pequeno

def alinhar():
    while(sensorc1.color()!=Color.BLACK or sensorc2.color()!=Color.BLACK):
        andar_igual(100)
        if(sensorc1.color()==Color.BLACK):  #alinhar com a linha preta
            while(sensorc2.color()!=Color.BLACK):
                motorB.hold()
                motorC.run(50)
            break
        if(sensorc2.color()==Color.BLACK):
            while(sensorc1.color()!=Color.BLACK):
                motorC.hold()
                motorB.run(50)
            break

def main():
    #alinhar()
    andar_igual_tempo(200,25000)

    motorD.reset_angle(0)
    motorD.run_target(200,-80,then=Stop.HOLD)
    angD = motorD.angle()
    motorD.reset_angle(0)
    motorD.run_target(200,-angD,then=Stop.HOLD)

    cor_bloco = andar_bloco()
    tam_bloco,ang = identifica_bloco()

    motorA.reset_angle(0)
    if(tam_bloco==0):
        motorB.reset_angle(0)
        motorB.run_target(200,-380,then=Stop.HOLD)
        motorA.run_angle(300,-ang,then=Stop.HOLD) #devolve o bloco pequeno
        motorB.run_target(200,0,then=Stop.HOLD)
    else:
        motorA.run_angle(300,300,then=Stop.HOLD) #termina de levantar o bloco grande
        anguloGarra=ang+300
        return

    motorB.reset_angle(0)
    motorB.run_target(200,380,then=Stop.HOLD)

main()

while True:
    if(sensorc1.reflection()<20 and sensorc2.reflection()<20):
        motorB.hold()
        motorC.hold()
        break
    if(sensorc1.reflection()<60):
        motorB.run(150*1.6)
        vB = motorB.speed()
        motorC.run(vB*0.625) #inverso da porcentagem
    if(sensorc1.reflection()>65):
        motorB.run(150)
        vB = motorB.speed()
        motorC.run(vB*1.6) #porcentagem de multiplicacao
    else:
        andar_igual(195) #media

motorC.reset_angle(0)
motorC.run_target(200,130,then=Stop.HOLD)
angC = motorC.angle()
andar_igual_tempo(200,1000)
motorC.reset_angle(0)
motorC.run_target(200,-angC,then=Stop.HOLD)
andar_igual_tempo(200,2000)
