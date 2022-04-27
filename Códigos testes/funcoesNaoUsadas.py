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
        if(sensorc1.rgb()[2]>2): 
            cor_bloco = Color.BLUE
            array.append(sensorc1.rgb()[2])
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
        if(check>=200): break
    ev3.speaker.beep()
    motorC.hold()
    motorB.hold()

    ang = motorB.angle()
    if(ang>100): tam_bloco = 1
    else: tam_bloco = 0

    array.append(tam_bloco)
    array.append(cor_bloco)
    
    def curva_giro(angulo): #angulo positivo: direita, negativo: esquerda
    sensorg4.reset_angle(0)  
    Kp = 5 
    Ki = 0.4
    Kd = 8
    
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
        motorC.run(-5-correcao)
        motorB.run(5+correcao)

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
        deriv = ((erro - erro0)*Kd)/tempoDecor

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

    angAt = 0
    t = 0
    integ = 0
    erro = 0
    sensorg4.reset_angle(0)
    cronometro.reset()
    motorB.reset_angle(0)
    while(motorB.angle()<graus):
        angAt = sensorg4.angle()
        erro0 = erro
        erro = -angAt

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
