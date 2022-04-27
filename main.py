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
#motorD = Motor(Port.D)
motorA = Motor(Port.A)
#sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)
sensorc3 = ColorSensor(Port.S3)
#sensorg4 = GyroSensor(Port.S4)
cronometro = StopWatch()

### MOVIMENTACAO ###

def segue_linha_c2(vel):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05 
    Kd = 10

    erro = 0
    valorI = 0
    t = 0
    check = 0
    cronometro.reset()
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

        if(cronometro.time()>2000): break
        #if(sensorc1.reflection()<5): check = check + 1 
        #else: check = 0   
        #if(check>=20): break     

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
    check = 0
    cronometro.reset()
    while True:
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

        if(cronometro.time()>2000): break
        #if(sensorc1.reflection()<5): check = check + 1 
        #else: check = 0   
        #if(check>=20): break     

    motorC.hold()
    motorB.hold()

def curva(angulo): #angulo positivo: direita, negativo: esquerda
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    Kp = 4
    Ki = 0.5
    Kd = 10
    
    t = 0
    integ = 0
    erro = 0
    while(motorB.angle()!=angulo):
        media = (motorB.angle() - motorC.angle())/2
        erro0 = erro
        erro = angulo - media

        prop = erro*Kp
        if(-3<erro<3): integ = integ+(erro*Ki)
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv
        motorC.run(-5-correcao)
        motorB.run(5+correcao)

    motorC.hold()
    motorB.hold()

def anda_reto_linha(velBase):
    Kp = 3 
    Ki = 0.02
    Kd = 3 

    t = 0
    integ = 0
    erro = 0
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        erro0 = erro
        erro = motorC.angle() - motorB.angle()

        prop = erro*Kp 
        if(-3<erro<3): integ = integ+(erro*Ki)
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv
        motorC.run(velBase-correcao)
        motorB.run(velBase+correcao)

    motorC.hold()
    motorB.hold()

def anda_reto_graus(velBase,graus):
    Kp = 3 
    Ki = 0.02
    Kd = 3 

    t = 0
    integ = 0
    erro = 0
    media = 0
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    cronometro.reset()
    while(media<graus):
        media = (motorB.angle() + motorC.angle())/2
        erro0 = erro
        erro = motorC.angle() - motorB.angle()

        prop = erro*Kp 
        if(-3<erro<3): integ = integ+(erro*Ki)
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv
        motorC.run(velBase-correcao)
        motorB.run(velBase+correcao)

    motorC.hold()
    motorB.hold()

def alinhar(vel):
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        motorC.run(vel)
        motorB.run(vel) #identifica a linha preta
    #ev3.speaker.beep()
    motorC.hold()
    motorB.hold()
    while(sensorc2.color()!=Color.BLACK):
        motorB.run(vel)
    motorB.hold()
    while(sensorc2.color()!=Color.WHITE): #alinha o motor B
        motorB.run(vel)
    motorB.reset_angle(0)
    #ev3.speaker.beep()
    motorB.hold()
    motorB.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc2.color()!=Color.WHITE):
        motorB.run(-vel)
    #ev3.speaker.beep()
    motorB.hold()
    angB = motorB.angle()
    motorB.reset_angle(0)
    motorB.run_target(vel,-angB/2,then=Stop.HOLD)

    while(sensorc3.color()!=Color.BLACK):
        motorC.run(vel)
    motorC.hold()
    while(sensorc3.color()!=Color.WHITE): #alinha o motor C
        motorC.run(vel)
    motorC.reset_angle(0)
    #ev3.speaker.beep()
    motorC.hold()
    motorC.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc3.color()!=Color.WHITE):
        motorC.run(-vel)
    #ev3.speaker.beep()
    motorC.hold()
    angC = motorC.angle()
    motorC.reset_angle(0)
    motorC.run_target(vel,-angB/2,then=Stop.HOLD)
        
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

### LOGICA ###

def identifica_bloco(array):
    cronometro.reset()
    check = 0
    while True:
        motorC.run(50)
        velC = motorC.speed()
        motorB.run(velC)
        if(sensorc1.reflection()>1):
            check = check + 1
        else: check = 0
        if(check>=20):
            motorC.hold()
            motorB.hold()
            anda_reto_graus(50,50) 
            tam_bloco = 1
            wait(100)
            if(sensorc2.reflection()>5):
                wait(10)
                if(sensorc2.rgb()[2]>3):
                    cor_bloco = Color.BLUE
                    ev3.speaker.beep(20)
            else:
                if(sensorc2.color()==Color.BLACK):
                    cor_bloco = Color.BLACK
                    ev3.speaker.beep(200)
            #array.append(sensorc1.rgb()[0])
            #array.append(sensorc1.rgb()[1])
            #array.append(sensorc1.rgb()[2])
            break
        if(cronometro.time()>5000): 
            tam_bloco = 0
            cor_bloco = sensorc1.color()
            break
    motorC.hold()
    motorB.hold()
    array.append(cor_bloco)
    array.append(tam_bloco)
            
def main():
    velG = 200
    relatorio = []

    segue_linha_c2(velG)
    curva(-220)
    alinhar(velG)
    deixa_bloco()
    anda_reto_graus(velG,-100)
    curva(-220)
    segue_linha_c3(velG)

    #segue_linha_c2(velG)
    #curva(-440)
    #segue_linha_c3(velG)
    #wait(5000)

    #alinhar(50)
    #anda_reto_graus(velG,300)
    #curva(145)
    #anda_reto_graus(velG,350)
    #curva(-145)

    #n = 0 
    #while(n!=3):
    #    anda_reto_linha(velG)
    #    alinhar(100)
    #    identifica_bloco(relatorio)
    #    wait(100)
    #    anda_reto_graus(velG,200)
    #    if(relatorio[(n*2)]==Color.BLUE): break
    #    n = n + 1

    #nSaida = n
    #curva(-220)
    #anda_reto_graus(velG,50)
    #motorA.reset_angle(0)
    #motorA.run_target(300,400,then=Stop.HOLD)
    #wait(500)
    #anda_reto_graus(velG,-50)
    #curva(220)

    #while(n!=3):
    #    anda_reto_linha(velG)
    #    alinhar(100)
    #    n = n + 1

    #motorA.reset_angle(0)
    #motorA.run_target(300,400,then=Stop.HOLD)
    #wait(500)
    #curva(220)
    #motorD.reset_angle(0)
    #motorD.run_target(300,400,then=Stop.BRAKE)
    #segue_linha_c2(velG)
    #motorD.run_target(300,0,then=Stop.BRAKE)
    #anda_reto_graus(velG,300)
    #curva(-220)
    #motorA.run_target(300,0,then=Stop.HOLD)

    for x in relatorio:
        print(x)

main()
