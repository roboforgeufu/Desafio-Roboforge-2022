def identifica_bloco():
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
        return 1 #bloco grande
    else:
        return 0 #bloco pequeno
