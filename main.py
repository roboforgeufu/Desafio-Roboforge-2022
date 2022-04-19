#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor,GyroSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.tools import StopWatch, wait
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
sensorg4 = GyroSensor(Port.S4)
cronometro = StopWatch()

def segue_linha_c2(vel):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05 
    Kd = 10
    erro = 0
    valorI = 0
    t = 0
    check = 0
    while True:
        erro0 = erro
        erro = valorLum - sensorc2.reflection()  

        valorP = erro*Kp

        if(-3<erro<3): valorI = (valorI+erro)*Ki

        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        valorD = ((erro - erro0)*Kd)/tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel-valorPID)
        motorB.run(vel+valorPID)

        if(sensorc1.color()==Color.BLACK): check = check + 1
        else: check = 0
        if(check>=2000): break

    motorC.hold()
    motorB.hold()

def segue_linha_c3(vel):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05
    Kd = 10
    erro = 0
    valorI = 0
    t = 0
    while(sensorc1.color()!=Color.BLUE):
        erro0 = erro
        erro = valorLum - sensorc3.reflection()  

        valorP = erro*Kp

        if(-3<erro<3): valorI = (valorI+erro)*Ki

        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        valorD = ((erro - erro0)*Kd)/tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel+valorPID)
        motorB.run(vel-valorPID)

    motorC.hold()
    motorB.hold()

def alinhar():
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        motorC.run(100)
        motorB.run(100) #identifica a linha preta
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    while(sensorc2.color()!=Color.BLACK):
        motorB.run(100)
    motorB.hold()
    while(sensorc2.color()!=Color.WHITE): #alinha o motor B
        motorB.run(100)
    motorB.reset_angle(0)
    ev3.speaker.beep()
    motorB.hold()
    motorB.run_target(100,-50,then=Stop.HOLD)
    while(sensorc2.color()!=Color.WHITE):
        motorB.run(-100)
    ev3.speaker.beep()
    motorB.hold()
    angB = motorB.angle()
    motorB.reset_angle(0)
    motorB.run_target(100,-angB/2,then=Stop.HOLD)

    while(sensorc3.color()!=Color.BLACK):
        motorC.run(100)
    motorC.hold()
    while(sensorc3.color()!=Color.WHITE): #alinha o motor C
        motorC.run(100)
    motorC.reset_angle(0)
    ev3.speaker.beep()
    motorC.hold()
    motorC.run_target(100,-50,then=Stop.HOLD)
    while(sensorc3.color()!=Color.WHITE):
        motorC.run(-100)
    ev3.speaker.beep()
    motorC.hold()
    angC = motorC.angle()
    motorC.reset_angle(0)
    motorC.run_target(100,-angB/2,then=Stop.HOLD)
        
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

def curva_giro(lado, angulo): #lado: 1 = sentido horario(direita)
    sensorg4.reset_angle(0)   #      0 = sentido anti-horario(esquerda)
    Kp = 5 
    Ki = 0.02
    Kd = 10 
    
    angAt = 0
    t = 0
    integ = 0
    erro = 0
    while(angAt!=angulo):
        angAt = sensorg4.angle()
        erro0 = erro
        erro = angulo - angAt

        prop = erro*Kp

        if(-3<erro<3): integ = integ+(erro*Ki)

        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv

        if(lado):
            motorC.run(-5-correcao)
            motorB.run(5+correcao)
        else:
            motorC.run(5+correcao)
            motorB.run(-5-correcao)

    motorC.hold()
    motorB.hold()

def anda_reto_linha(velBase):
    Kp = 3 
    Ki = 0.02
    Kd = 3 

    angAt = 0
    t = 0
    integ = 0
    erro = 0
    sensorg4.reset_angle(0)
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        angAt = sensorg4.angle()
        erro0 = erro
        erro = -angAt

        prop = erro*Kp 

        if(-3<erro<3): integ = integ+(erro*Ki)

        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd) 

        correcao = prop+integ+deriv

        motorC.run(velBase-correcao)
        motorB.run(velBase+correcao)
    motorC.hold()
    motorB.hold()

def anda_reto_tempo(velBase,tempo):
    Kp = 3 
    Ki = 0.02
    Kd = 3 

    angAt = 0
    t = 0
    integ = 0
    erro = 0
    sensorg4.reset_angle(0)
    cronometro.reset()
    while(cronometro.time()<tempo):
        angAt = sensorg4.angle()
        erro0 = erro
        erro = -angAt

        prop = erro*Kp 

        if(-3<erro<3): integ = integ+(erro*Ki)

        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd) 

        correcao = prop+integ+deriv

        motorC.run(velBase-correcao)
        motorB.run(velBase+correcao)
    motorC.hold()
    motorB.hold()

def identifica_bloco(array):
    while True:
        motorC.run(50)
        motorB.run(50)
        if(sensorc1.color()==Color.BLACK or sensorc1.color()==Color.BLUE or sensorc1.color()==Color.RED): 
            check = check + 1
        else: check = 0
        if(check>=20): break
    wait(100)
    cor_bloco = sensorc1.color()
    if(cor_bloco == Color.BLACK):
        if(sensorc1.rgb()[2]>0): cor_bloco = Color.BLUE
    if(cor_bloco == Color.BROWN): cor_bloco = Color.RED
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    motorB.reset_angle(0)

    while True:
        motorC.run(50)
        motorB.run(50)
        if(sensorc1.color()!=Color.BLACK and sensorc1.color()!=Color.BLUE and sensorc1.color()!=Color.RED): 
            check = check + 1
        else: check = 0
        if(check>=500): break
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()

    ang = motorB.angle()
    if(ang>100): tam_bloco = 1
    else: tam_bloco = 0

    array.append(tam_bloco)
    array.append(cor_bloco)

def main():
    relatorio = []
    anda_reto_linha(200)
    alinhar()
    identifica_bloco(relatorio)
    anda_reto_linha(200)
    alinhar()
    identifica_bloco(relatorio)
    anda_reto_linha(200)
    alinhar()
    identifica_bloco(relatorio)

    for x in relatorio:
        print(x)

main()
