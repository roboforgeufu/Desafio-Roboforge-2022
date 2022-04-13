#def andar_igual_tempo(vel,time):
#    cronometro.reset()
#    ajuste = 0
#    while(cronometro.time()<time):
#        if(ajuste<1):
#            ajuste = ajuste + 0.002
#        motorC.run(vel*ajuste)
#        vC = motorC.speed()
#        motorB.run(vC) #joga o valor de vel lido para o segundo motor
#    motorB.hold()
#    motorC.hold()

#def andar_bloco():
#    while(sensorc4.reflection()<10): #vai para frente até o bloco tampar o sensor
#        motorB.run(50)
#        vB = motorB.speed()
#        motorC.run(vB)
#    andar_igual_tempo(-50,400)
#    cor = sensorc3.color() #le a cor do bloco
#    andar_igual_tempo(-50,2600)
#    return cor

#def identifica_bloco():
#    mediaV=330 #velocidade media no duty cycle referido (340 bateria cheia)
#    n=0
#    media=[]
#    motorA.reset_angle(0)
#    while True:
#        motorA.dc(40)
#        velA=motorA.speed()
#        #print(velA)
#        if(velA>310): # 320 bateria cheia
#            media.append(velA) #joga os valores válidos (sem ruído) em uma lista
#            if(len(media)==5): #quando a lista atinge 5 valores calcula a média entre eles
#                for i in range(5): 
#                    mediaV = mediaV + media[i]
#                mediaV = mediaV/6
#                media.clear()
#                n=n+1
#        if(n>10 and mediaV<315): #descarta os 50 primeiros valores válidos (aceleração incial) antes de comparar
#            break
#    angGarra = motorA.angle()
#    if(angGarra<530):
#        return 1,angGarra #bloco grande
#    else:
#        return 0,angGarra #bloco pequeno
