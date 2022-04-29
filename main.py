#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.tools import StopWatch, wait
# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motorB = Motor(Port.B)
motorC = Motor(Port.C)
motorA = Motor(Port.A)
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)
sensorc3 = ColorSensor(Port.S3)
sensorc4 = ColorSensor(Port.S4)
cronometro = StopWatch()

### MOVIMENTACAO ###

def segue_linha_c2(vel,tempo):
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

        if(cronometro.time()>tempo): break
    motorC.hold()
    motorB.hold()

def segue_linha_c3(vel,tempo):
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

        if(cronometro.time()>tempo): break
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
        vel = 5 + correcao
        if(vel<0):
            if(vel>-5): vel = -5
        else:
            if(vel<5): vel = 5
        motorC.run(-vel)
        motorB.run(vel)

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

def anda_reto_graus(velBase,graus,stop): #para dar ré os dois valores devem ser negativos
    Kp = 3                               #stop 1: hold 2: brake 3: coast
    Ki = 0.02
    Kd = 3 

    t = 0
    integ = 0
    erro = 0
    media = 0
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    cronometro.reset()
    if(graus<0):
        while(media>graus):
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
    else:
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
    if(stop==1):
        motorC.hold()
        motorB.hold()
    if(stop==2):
        motorC.brake()
        motorB.brake()
    if(stop==3):
        return

def alinhar(vel):
    while(sensorc2.color()!=Color.BLACK and sensorc3.color()!=Color.BLACK):
        motorC.run(vel)
        motorB.run(vel) #identifica a linha preta

    motorC.hold()
    motorB.hold()

    while(sensorc3.color()!=Color.BLACK):
        motorC.run(vel)
    motorC.hold()
    while(sensorc3.color()!=Color.WHITE): #alinha o motor C
        motorC.run(vel)
    motorC.reset_angle(0)

    motorC.hold()
    motorC.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc3.color()!=Color.WHITE):
        motorC.run(-vel)

    while(sensorc2.color()!=Color.BLACK):
        motorB.run(vel)
    motorB.hold()
    while(sensorc2.color()!=Color.WHITE): #alinha o motor B
        motorB.run(vel)
    motorB.reset_angle(0)

    motorB.hold()
    motorB.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc2.color()!=Color.WHITE):
        motorB.run(-vel)

    motorB.hold()
    angB = motorB.angle()
    motorB.reset_angle(0)
    motorB.run_target(vel,-angB/2,then=Stop.HOLD)

    FC = 0.8 #fator de correcao
    motorC.hold()
    angC = motorC.angle()
    motorC.reset_angle(0)
    motorC.run_target(vel,(-angB/2)*FC,then=Stop.HOLD)
        
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
    while(cronometro.time()<3300): motorA.dc(-40)
    motorA.hold()

def pega_bloco(): 
    cronometro.reset()
    while(cronometro.time()<4000): motorA.dc(80-(cronometro.time()*0.022))
    motorA.hold()
    wait(500)

def alinhar_linha(vel): ###
    while(sensorc3.color()!=Color.BLACK):
        motorC.run(vel)
        motorB.run(-vel)
    motorC.hold()
    motorB.hold()
    motorC.reset_angle(0)
    motorB.reset_angle(0)
    while(sensorc2.color()!=Color.BLACK):
        motorC.run(-vel)
        motorB.run(vel)
    ang = (motorC.angle() - motorB.angle())/2
    motorC.hold()
    motorB.hold()
    curva(ang/2)


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
            anda_reto_graus(50,50,1) 
            tam_bloco = 1
            wait(100)
            if(sensorc1.reflection()>5):
                wait(10)
                if(sensorc1.rgb()[2]>3):
                    cor_bloco = Color.BLUE
                    ev3.speaker.beep(20)
                else: cor_bloco = Color.BLACK
            else:
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
    velG = 300
    relatorio = [] 

    alinhar(50)
    motorA.reset_angle(0)
    motorA.run_target(300,-200)
    anda_reto_graus(velG,300,1)
    curva(145)
    anda_reto_graus(velG,300,1)
    curva(-145)
    n = 0 
    flag = 0
    while(n!=3):
        anda_reto_linha(velG)
        alinhar(velG/3)
        identifica_bloco(relatorio)
        wait(100)
        if(relatorio[(n*2)]==Color.BLACK): 
            flag = 1
            pos = n
        n = n + 1
        if(n!=3): anda_reto_graus(velG,200,3)
    
    if(flag):
        n = 3
        while(n!=pos):
            anda_reto_linha(-velG)
            alinhar(velG/3)
            n = n - 1
            if(n!=pos): anda_reto_graus(-velG,-200,3)
        anda_reto_graus(velG,100,1)
        curva(-230)
        anda_reto_graus(-velG,-30,1)
        pega_bloco()
        anda_reto_graus(-velG,-60,1)
        curva(-230)
        while(n!=0):
            anda_reto_linha(velG)
            alinhar(velG/3)
            anda_reto_graus(velG,200,3)
            n = n - 1
        anda_reto_graus(velG,580,1)
        curva(-220)
        segue_linha_c3(velG,1000)
        curva(220)
        alinhar(velG/2)
        deixa_bloco()
        anda_reto_graus(-velG,-100,1)
        curva(440)
        relatorio[pos*2] = 'P_ent' #bloco preto entregue

        pos = 0
        n = 0
        for x in range(2,-1,-2):
            if(relatorio[x]==Color.BLACK):
                pos = x
                if(pos==0): n = 2
                if(pos==2): n = 6
                while(n!=pos):
                    anda_reto_linha(velG)
                    alinhar(velG/3)
                    n = n - 2
                    if(n!=pos): anda_reto_graus(velG,200,3)
                anda_reto_graus(velG,115,1)
                curva(-215)
                wait(100)
                while(sensorc4.ambient()>0):
                    motorC.run(velG)
                    velC = motorC.speed()
                    motorB.run(velC)
                motorC.hold()
                motorB.hold()
                anda_reto_graus(-velG,-80,1)
                pega_bloco()
                anda_reto_graus(-velG,-200,1)
                curva(-225)
                if(pos==2):
                    anda_reto_linha(velG)
                    alinhar(velG/3)
                if(pos==0):
                    anda_reto_graus(-velG,-200,3)
                    anda_reto_linha(velG)
                    alinhar(velG/3)
                anda_reto_graus(velG,750,1)
                curva(-220)
                if(pos==2): segue_linha_c3(velG,3200)
                if(pos==0): segue_linha_c3(velG,5000)
                curva(220)
                alinhar(velG/2)
                deixa_bloco()
                anda_reto_graus(-velG,-80,1)
                curva(220)
                if(pos==2): segue_linha_c2(velG,2200)
                if(pos==0): segue_linha_c2(velG,4200)
                curva(220)
                relatorio[pos] = 'P_ent'

    
        n = 0 
        flag = 0
        while(n!=3):
            anda_reto_linha(velG)
            alinhar(velG/3)
            n = n + 1
            if(n!=3): anda_reto_graus(velG,200,3)

    for x in range(4,-1,-2):
        if(relatorio[x]==Color.BLUE):
            pos = x/2
            n = 3
            while(n!=pos):
                anda_reto_linha(-velG)
                alinhar(velG/3)
                n = n - 1
                if(n!=pos): anda_reto_graus(-velG,-200,3)
            anda_reto_graus(velG,100,1)
            curva(-215)
            while(sensorc4.ambient()>0):
                motorC.run(velG)
                velC = motorC.speed()
                motorB.run(velC)
            motorC.hold()
            motorB.hold()
            anda_reto_graus(-velG,-80,1)
            pega_bloco()
            anda_reto_graus(-velG,-60,1)
            curva(220)
            n = n + 1
            while(n!=3):
                anda_reto_linha(velG)
                alinhar(velG/3)
                n = n + 1
                if(n!=3): anda_reto_graus(velG,150,3)
            if(pos==2):
                anda_reto_graus(-velG,-150,1)
                alinhar(velG/3)
            anda_reto_graus(velG,250,1)
            curva(220)
            segue_linha_c2(velG,1200)
            curva(-220)
            alinhar(velG/2)
            deixa_bloco()
            anda_reto_graus(-velG,-100,1)
            curva(440)
            relatorio[int(pos)] = 'A_ent' #bloco azul entregue
            break

    pos = 0
    n = 0
    for x in range(2,-1,-2):
        if(relatorio[x]==Color.BLUE):
            pos = x
            n = 4
            anda_reto_graus(-velG,-100,1)
            while(n!=pos):
                anda_reto_linha(velG)
                alinhar(velG/3)
                n = n - 2
                if(n!=pos): anda_reto_graus(velG,200,3)
            anda_reto_graus(velG,115,1)
            curva(225)
            wait(100)
            while(sensorc4.ambient()>0):
                motorC.run(velG)
                velC = motorC.speed()
                motorB.run(velC)
            motorC.hold()
            motorB.hold()
            anda_reto_graus(-velG,-80,1)
            pega_bloco()
            anda_reto_graus(-velG,-200,1)
            curva(225)
            if(pos==2):
                anda_reto_linha(velG)
                alinhar(velG/3)
            if(pos==0):
                anda_reto_linha(velG)
                alinhar(velG/3)
                anda_reto_linha(velG)
                alinhar(velG/3)
            anda_reto_graus(velG,250,1)
            curva(220)
            if(pos==2): segue_linha_c2(velG,3200)
            if(pos==0): segue_linha_c2(velG,5000)
            curva(-220)
            alinhar(velG/2)
            deixa_bloco()
            anda_reto_graus(-velG,-80,1)
            curva(-220)
            if(pos==2): segue_linha_c3(velG,2200)
            if(pos==0): segue_linha_c3(velG,4200)
            curva-(220)

main()
