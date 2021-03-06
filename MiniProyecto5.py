#Universidad del Valle de Guatemala
#David Soto #17551
#Miguel Valle #17102

import numpy as np
import random
import math
import pygame

CLOCKWISE = -1
COUNTERCLOCKWISE = 1

### Puntos para las funciones de entrada Crisp
PUNTOS_ESTA_CERCA = [(0,1),(10,1),(190,0),(200,0)]
PUNTOS_ESTA_LEJOS = [(0,0),(10,0),(190,1),(200,1)]

PUNTOS_ESTA_VIENDO_DIRECTO = [(0,1),(20,1),(160,0),(180,0)]
PUNTOS_ESTA_VIENDO_LADO_CONTRARIO = [(0,0),(20,0),(160,1),(180,1)]

### Puntos para las funciones de salida Crisp
PUNTOS_MOVERSE_POCO = [(0,1),(2,1),(4,0),(5,0)]
PUNTOS_MOVERSE_MUCHO = [(0,0),(2,0),(4,1),(5,1)]

PUNTOS_ROTAR_POCO = [(0,1),(3,1),(7,0),(10,0)]
PUNTOS_ROTAR_MUCHO = [(0,0),(3,0),(7,1),(10,1)]


### Este metodo nos dira el angulo que deberia de girar el robot para ir directo al objetivo
### Tambien nos dice la direccion del giro siendo -1 CLOCKWISE y 1 COUNTERCLOCKWISE
### El metodo recibe la posicion del robot, la posicion de la pelora y el angulo al cual esta viendo el robot
### Devuelve la distancia a girar y el sentido del giro
def calcularAnguloGiro(pRobot, pPelota, aRobot):
    xr, yr = pRobot
    xp, yp = pPelota

    dy = yp - yr
    dx = xp - xr

    aDireccion = math.atan2(dy, dx)
    dGiroDerecha = 0
    dGiroIzquierda = 0

    if aDireccion < 0:
        aDireccion = math.degrees(aDireccion + 2 * math.pi)
    else:
        aDireccion = math.degrees(aDireccion)

    ### Se revisa la distancia del angulo girando a la derecha
    if  aRobot >= aDireccion:
        dGiroDerecha = aRobot - aDireccion
    else:
        dGiroDerecha = aRobot - (aDireccion - 360)

    ### Se revisa la distancia del angulo girando a la izquierda
    if  aDireccion >= aRobot:
        dGiroIzquierda = aDireccion - aRobot
    else:
        dGiroIzquierda = aDireccion - (aRobot - 360)

    ### Se devuelve el tamaño del angulo y 
    if dGiroDerecha <= dGiroIzquierda:
        return dGiroDerecha, CLOCKWISE
    else:
        return dGiroIzquierda, COUNTERCLOCKWISE

### Este metodo nos da la distancia entre 2 puntos
### Recibe 2 puntos
### Devuelve la distancia
def calcularDistanciaPuntos(punto1, punto2):
    x1, y1 = punto1
    x2, y2 = punto2

    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

### Este metodo calcula el X y Y al que se debe mover el robot
### Dado una distancia a recorrer y un angulo de trayectoria
### Devuelve los valores X y Y
def calcularXYDesplazar(distancia, angulo):
    x = distancia * math.cos(math.radians(angulo))
    y = distancia * math.sin(math.radians(angulo))

    return x, y

### Es metodo calcula el angulo al que debe de disparar el robot para
### marcar un gol desde su posición al centro de la porteria y tambien
### devuelve la distancia a recorrer en el eje X, con la posicion del robot y de la porteria
def calcularAnguloDisparoYDistanciaX(pRobot, pPorteria):
    xr, yr = pRobot
    xp, yp = pPorteria

    dy = yp - yr
    dx = xp - xr

    aDireccion = math.atan2(dy, dx)
    dGiroDerecha = 0
    dGiroIzquierda = 0

    if aDireccion < 0:
        aDireccion = math.degrees(aDireccion + 2 * math.pi)
    else:
        aDireccion = math.degrees(aDireccion)

    return aDireccion, abs(xr - xp)

### Calculamos el valor Y de un tiro a cierto angulo a cierta distancia X de la porteria
### X deberia ser la distancia horizontal entre la porteria y el robot
### Devuelve el punto en Y que llegaria la pelota
def lanzarTiroCalculandoY(x, angulo):
    return x * math.tan(math.radians(angulo))

### Metodo que devuelve una funcion matematica definida por partes
### Lo que recibe son 4 puntos para determinar la forma de la funcion
def generador_funciones_2d(puntos):
    ### Se hace la logica para hacer las funciones ya determinada por
    x1, y1 = puntos[0]
    x2, y2 = puntos[1]
    x3, y3 = puntos[2]
    x4, y4 = puntos[3]

    ### Para el primer tramo
    m1 = (y2 * 1.0 - y1 * 1.0) / (x2 * 1.0 - x1 * 1.0)
    b1 = y2 - m1 * x2

    ### Para el segundo tramo
    m2 = (y3 * 1.0 - y2 * 1.0) / (x3 * 1.0 - x2 * 1.0)
    b2 = y3 - m2 * x3

    ### Para el tercer tramo
    m3 = (y4 * 1.0 - y3 * 1.0) / (x4 * 1.0 - x3 * 1.0)
    b3 = y4 - m3 * x4

    def inner_funcion_2d(d):
        if d > x3:
            return d * m3 + b3
        elif d > x2:
            return d * m2 + b2
        else:
            return d * m1 + b1

    return inner_funcion_2d

### Definicion de Clausulas de HORN
### Vienen dadas para primero usar una funcion de distancia y luego una de rotacion
def hornOR(fa1, fa2, fc3):
    def inner_hornOR(ds,rs,es):
        return min(max(fa1(ds), fa2(rs)), fc3(es))
    return inner_hornOR

def hornAND(fa1, fa2, fc3):
    def inner_hornAND(ds,rs,es):
        return min(min(fa1(ds), fa2(ds)), fc3(es))
    return inner_hornAND

### Definicion de las Funciones INPUT Crisp de distancia
estar_cerca =  generador_funciones_2d(PUNTOS_ESTA_CERCA)
estar_lejos = generador_funciones_2d(PUNTOS_ESTA_LEJOS)

### Definicion de las Funciones INPUT Crisp de rotacion
estar_viendo_directo = generador_funciones_2d(PUNTOS_ESTA_VIENDO_DIRECTO)
estar_viendo_lado_contrario = generador_funciones_2d(PUNTOS_ESTA_VIENDO_LADO_CONTRARIO)

### Definicion de las Funciones OUTPUT Crisp de distancia
moverse_poco = generador_funciones_2d(PUNTOS_MOVERSE_POCO)
moverse_mucho = generador_funciones_2d(PUNTOS_MOVERSE_MUCHO)

### Definicion de las Funciones OUTPUT Crisp de rotacion
rotar_poco = generador_funciones_2d(PUNTOS_ROTAR_POCO)
rotar_mucho = generador_funciones_2d(PUNTOS_ROTAR_MUCHO)

### Definicion de Clausulas de HORN para la Distancia
### Si estoy cerca o estoy viendo al lado opuesto, moverse poco
hornD1 = hornOR(estar_cerca, estar_viendo_lado_contrario, moverse_poco)
### Si estoy lejos o estoy viendo directamente, moverse mucho
hornD2 = hornOR(estar_lejos, estar_viendo_directo, moverse_mucho)

### Definicion de Clausulas de HORN para la Rotacion
### Si estoy lejos o estoy viendo al lado opuesto, rotar mucho
hornR1 = hornOR(estar_lejos, estar_viendo_lado_contrario, rotar_mucho)
### Si estoy cerca o estoy viendo directamente, rotar poco
hornR2 = hornOR(estar_cerca, estar_viendo_directo, rotar_poco)

### Definimos el OUTPUT Difuso para la distancia
output_difuso_distancia = lambda ds, rs, es : max(
    hornD1(ds,rs,es),
    hornD2(ds,rs,es)
)

### Definimos el OUTPUT Difuso para la rotacion
output_difuso_rotacion = lambda ds, rs, es : max(
    hornR1(ds,rs,es),
    hornR2(ds,rs,es)
)

def fuzzyDistancia(dPelota, rPelota):
    xs = np.linspace(0, 5, 100)
    ys = [output_difuso_distancia(dPelota, rPelota, x) for x in xs]

    return np.sum(xs * ys) / np.sum(ys) 

def fuzzyRotacion(dPelota, rPelota):
    xs = np.linspace(0, 10, 100)
    ys = [output_difuso_distancia(dPelota, rPelota, x) for x in xs]

    return np.sum(xs * ys) / np.sum(ys) 

### Main para la simulacion
ANCHO_PORTERIA = 15
LARGO_CANCHA = 200
ANCHO_CANCHA = 100
METIO_GOL = False

POSICION_INICIAL_ROBOT = (random.randint(0,LARGO_CANCHA),random.randint(0,ANCHO_CANCHA))
POSICION_INICIAL_PELOTA = (random.randint(0,LARGO_CANCHA),random.randint(0,ANCHO_CANCHA))
DIRECCION_INICIAL_ROBOT = random.randint(0,359)
POSICION_INICIAL_PORTERIA = (200, 50)

# Datos Pygame
pygame.init()
screen = pygame.display.set_mode((LARGO_CANCHA * 3, ANCHO_CANCHA * 3))
pygame.display.set_caption("Mini_Proyecto5")
icon = pygame.image.load('video-game.png')
campo = pygame.image.load('test.png')
ball = pygame.image.load('football-ball.png')
playerOG = pygame.image.load('right-arrow.png')
player = playerOG
ball = pygame.transform.scale(ball, (int(ANCHO_CANCHA / 10) * 3, int(ANCHO_CANCHA / 10) * 3))
player = pygame.transform.scale(player, (int(ANCHO_CANCHA / 10) * 3, int(ANCHO_CANCHA / 10) * 3))
campo = pygame.transform.scale(campo, (LARGO_CANCHA * 3, ANCHO_CANCHA * 3))

ballX = POSICION_INICIAL_PELOTA[0]
ballY = POSICION_INICIAL_PELOTA[1]

playerX = POSICION_INICIAL_ROBOT[0]
playerY = POSICION_INICIAL_ROBOT[1]

playerA = DIRECCION_INICIAL_ROBOT


### Funciones para Pygame
def moverJugador(x1, y1, x2, y2):
    global playerX
    global playerY
    #print("Original Moviendo Robot x1,y1 = (", x1, ",", y1, ") a (", x2, ",", y2, ")")
    x2 = round(x2, 1) * 3
    y2 = round(y2, 1) * 3
    x1 = round(x1, 1) * 3
    y1 = round(y1, 1) * 3
    if (abs(x2-x1) > 2 and abs(y2-y1) > 2):
        print("Moviendo Robot x1,y1 = (", x1, ",", y1, ") a (", x2, ",", y2, ")")
        stepX = 0
        stepY = 0

        difx = x2 - x1
        dify = y2 - y1
        c = math.sqrt((dify) ** 2 + (difx) ** 2)
        decX = 1
        decY = 1

        tempX = cantCeros(difx / c)
        tempX += 1
        decX = (difx / c) / (10 ** (tempX - 1))

        tempY = cantCeros(dify / c)
        tempY += 1
        decY = (dify / c) / (10 ** (tempY - 1))

        if (x2 > x1):
            stepX = abs(decX)
            condX = 0
        elif (x2 < x1):
            stepX = abs(decX) * -1
            condX = 1

        if (y2 > y1):
            stepY = abs(decY)
            condY = 0
        elif (y2 < y1):
            stepY = abs(decY) * -1
            condY = 1

        verificacionX = False
        verificacionY = False
        print("Using stepX:", stepX, " stepY:", stepY)
        print("Using tempX:", tempX - 1, " tempY:", tempY - 1)
        # while((round(x1,tempX-1) != round(x2,tempX-1)) or (round(y1,tempY-1) != round(y2,tempY-1))):
        while (not (verificacionX) or not (verificacionY)):
            # print(round(x1, tempX - 1), ",", round(y1, tempY - 1), " to ", round(x2, tempX - 1), ",", round(y2, tempY - 1))
            # if(round(x1,tempX-1) == round(x2,tempX-1)):
            if ((condX == 0 and x1 >= x2) or (condX == 1 and x1 <= x2)):
                stepX = 0
                verificacionX = True
            # if(round(y1,tempY-1) == round(y2,tempY-1)):
            if ((condY == 0 and y1 >= y2) or (condY == 1 and y1 <= y2)):
                stepY = 0
                verificacionY = True
            x1 += stepX
            y1 += stepY
            screen.blit(campo, (0, 0))
            screen.blit(ball, (ballX*3, ballY*3))
            screen.blit(player, (x1, y1))
            pygame.display.update()
        playerX = x1 / 3
        playerY = y1 / 3
        print("Se movio el robot")


def moverPelota(x1, y1, x2, y2):
    global ballX
    global ballY
    print("Original Moviendo pelota x1,y1 = (", x1, ",", y1, ") a (", x2, ",", y2, ")")
    x2 = round(x2, 1) * 3
    y2 = round(y2, 1) * 3
    x1 = round(x1, 1) * 3
    y1 = round(y1, 1) * 3
    if (abs(x2-x1) > 2 and abs(y2-y1) > 2):
        print("Moviendo pelota x1,y1 = (", x1, ",", y1, ") a (", x2, ",", y2, ")")
        stepX = 0
        stepY = 0

        difx = x2 - x1
        dify = y2 - y1
        c = math.sqrt((dify) ** 2 + (difx) ** 2)
        decX = 1
        decY = 1

        tempX = cantCeros(difx / c)
        tempX += 1
        decX = (difx / c) / (10 ** (tempX - 1))

        tempY = cantCeros(dify / c)
        tempY += 1
        decY = (dify / c) / (10 ** (tempY - 1))

        if (x2 > x1):
            stepX = abs(decX)
            condX = 0
        elif (x2 < x1):
            stepX = abs(decX) * -1
            condX = 1

        if (y2 > y1):
            stepY = abs(decY)
            condY = 0
        elif (y2 < y1):
            stepY = abs(decY) * -1
            condY = 1

        verificacionX = False
        verificacionY = False
        print("Using stepX:", stepX, " stepY:", stepY)
        print("Using tempX:", tempX - 1, " tempY:", tempY - 1)
        # while((round(x1,tempX-1) != round(x2,tempX-1)) or (round(y1,tempY-1) != round(y2,tempY-1))):
        while (not (verificacionX) or not (verificacionY)):
            # print(round(x1,tempX-1),",",round(y1,tempY-1)," to ",round(x2,tempX-1),",",round(y2,tempY-1))
            # if(round(x1,tempX-1) == round(x2,tempX-1)):
            if ((condX == 0 and x1 >= x2) or (condX == 1 and x1 <= x2)):
                stepX = 0
                verificacionX = True
            # if(round(y1,tempY-1) == round(y2,tempY-1)):
            if ((condY == 0 and y1 >= y2) or (condY == 1 and y1 <= y2)):
                stepY = 0
                verificacionY = True
            x1 += stepX
            y1 += stepY
            screen.blit(campo, (0, 0))
            screen.blit(ball, (x1, y1))
            screen.blit(player, (playerX*3, playerY*3))
            pygame.display.update()
        ballX = x1 / 3
        ballY = y1 / 3
        print("Se movio la pelota")

def rotarJugador(inicio, fin):
    print(inicio,":",fin)
    global player
    global playerA
    global rect
    inicio = round(inicio)
    fin = round(fin)
    if fin < 0:
        fin = 360 - fin
    #inicio += -90
    #fin += -90
    if (inicio <= fin):
        step = 1
    if (inicio > fin):
        step = -1
    turn = True
    temp = 0
    while (turn):
        inicio = (inicio + step) % 361
        player = pygame.transform.rotate(playerOG, inicio)
        screen.blit(campo, (0, 0))
        temp = player.get_rect()
        temp.center = (playerX * 3, playerY * 3)
        screen.blit(ball, (ballX * 3, ballY * 3))
        screen.blit(player, temp)
        #print(inicio)
        if ((step == 1 and round(inicio) >= fin) or (step == -1 and round(inicio) <= fin)):
            turn = False
        pygame.display.update()
    playerA = inicio



def juegoInicio():
    print(POSICION_INICIAL_PELOTA)
    player = pygame.transform.rotate(playerOG, DIRECCION_INICIAL_ROBOT-90)
    screen.blit(campo, (0, 0))
    screen.blit(ball, (POSICION_INICIAL_PELOTA[0]*3,POSICION_INICIAL_PELOTA[1]*3))
    screen.blit(player, (POSICION_INICIAL_ROBOT[0]*3, POSICION_INICIAL_ROBOT[1]*3))
    pygame.display.update()


def cantCeros(x):
    x = str(x)
    # print(x)
    temp = x.split("0.")
    # print(temp[1])
    c = 0
    for i in temp:
        if (i == "0"):
            c += 1
    return c


### Inicio Juego

juegoInicio()

### Prmero se calcula la distancia y rotacion a la que estamos
DISTANCIA_ACTUAL = calcularDistanciaPuntos(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PELOTA)

### CICLO HASTA QUE EL ROBOT LLEGUE A LA PELOTA A MENOS DE 10 PIXELES
while (DISTANCIA_ACTUAL > 10) or (not METIO_GOL):
    ### Obtencion de INPUTS
    distanciaARecorrer = calcularDistanciaPuntos(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PELOTA)
    anguloARotar, sentido = calcularAnguloGiro(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PELOTA, DIRECCION_INICIAL_ROBOT)

    ### Obtencion de resultados FUZZY
    distancia = fuzzyDistancia(distanciaARecorrer, anguloARotar)
    rotacion = fuzzyRotacion(distanciaARecorrer, anguloARotar)

    ### Una vez se obtengan los resultados se aplican la distancia y la rotacion a las variables control INICIAL
    posicionFinalX = 0
    posicionFinalY = 0

    ### Actualizacion de la posicion
    x, y = calcularXYDesplazar(distancia, ((DIRECCION_INICIAL_ROBOT + sentido * rotacion) % 360))
    posicionActualRobotX, posicionActualRobotY = POSICION_INICIAL_ROBOT
    
    if (x + posicionActualRobotX) > LARGO_CANCHA:
        posicionFinalX = LARGO_CANCHA
    elif (x + posicionActualRobotX) < 0:
        posicionFinalX = 0
    else:
        posicionFinalX = x + posicionActualRobotX

    if (y + posicionActualRobotY) > ANCHO_CANCHA:
        posicionFinalY = ANCHO_CANCHA
    elif (y + posicionActualRobotY) < 0:
        posicionFinalY = 0
    else:
        posicionFinalY = y + posicionActualRobotY

    ### Actualizacion de la posicion
    POSICION_INICIAL_ROBOT = (posicionFinalX, posicionFinalY)

    ### Actualizacion de la rotacion
    DIRECCION_INICIAL_ROBOT = (DIRECCION_INICIAL_ROBOT + sentido * rotacion) % 360

    ### Calcular si ya podemos salir del ciclo
    DISTANCIA_ACTUAL = calcularDistanciaPuntos(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PELOTA)

    ### EN LA UI
    ### En teoria aqui se utilizan los datos actualizados de posicion para mover al robot en la simulacion MIGUEL


    ### Parte Deterministica para lanzar el balon una vez el robot llegue a la pelota
    if  DISTANCIA_ACTUAL <= 10:
        ### VARIABLE DEL NUEVO ANGULO AL QUE DEBE DE VER -> DIRECCION_INICIAL_ROBOT
        rotarJugador(playerA, DIRECCION_INICIAL_ROBOT)
        ### VARIABLE DE LA SIGUIENTE POSICION -> POSICION_INICIAL_ROBOT
        moverJugador(playerX, playerY, POSICION_INICIAL_PELOTA[0], POSICION_INICIAL_PELOTA[1])
        print('TIRANDO A PORTERIA...')
        POSICION_INICIAL_ROBOT = POSICION_INICIAL_PELOTA
        DIRECCION_INICIAL_ROBOT, DISTANCIA_DISPARO_X = calcularAnguloDisparoYDistanciaX(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PORTERIA)

        ### Aqui se calcula que tanto se va a desviar con distribucion Normal
        DIRECCION_INICIAL_ROBOT = DIRECCION_INICIAL_ROBOT + np.random.normal(0,5)

        ### Calcular a donde va a ir a dar la pelota en el eje Y
        DESPLAZAMIENTO_Y_PELOTA = lanzarTiroCalculandoY(DISTANCIA_DISPARO_X, DIRECCION_INICIAL_ROBOT)

        ### Setear la posicion final de la pelota
        if (POSICION_INICIAL_PELOTA[1] +  DESPLAZAMIENTO_Y_PELOTA) > ANCHO_CANCHA:
            POSICION_INICIAL_PELOTA = (200, ANCHO_CANCHA)
        elif (POSICION_INICIAL_PELOTA[1] +  DESPLAZAMIENTO_Y_PELOTA) < 0:
            POSICION_INICIAL_PELOTA = (200, 0)
        else:
            POSICION_INICIAL_PELOTA = (200, POSICION_INICIAL_PELOTA[1] +  DESPLAZAMIENTO_Y_PELOTA)

        ### EN LA UI MIGUEL
        ### PONER A VER AL ROBOT EN LA DIRECCION {DIRECCION_INICIAL_ROBOT}
        rotarJugador(playerA, DIRECCION_INICIAL_ROBOT)
        ### POSICIONAR ROBOT EN {POSICION_INICIAL_ROBOT}
        moverJugador(playerX, playerY, POSICION_INICIAL_ROBOT[0], POSICION_INICIAL_ROBOT[1])
        ### MOVER LA PELOTA A {POSICION_INICIAL_PELOTA}
        moverPelota(ballX, ballY, POSICION_INICIAL_PELOTA[0], POSICION_INICIAL_PELOTA[1])

        ### Si echa gol terminar simulacion
        if (POSICION_INICIAL_PELOTA[1] > (POSICION_INICIAL_PORTERIA[1] - ANCHO_PORTERIA)) and (POSICION_INICIAL_PELOTA[1] < (POSICION_INICIAL_PORTERIA[1] + ANCHO_PORTERIA)):
            print('GOOOOLLL')
            METIO_GOL = True
        else:
            print('AFUERAAA')
            ### Se colocan nuevamente random el robot, la pelota y el angulo al que ve el robot
            POSICION_INICIAL_ROBOT = (random.randint(0,LARGO_CANCHA),random.randint(0,ANCHO_CANCHA))
            POSICION_INICIAL_PELOTA = (random.randint(0,LARGO_CANCHA),random.randint(0,ANCHO_CANCHA))
            DIRECCION_INICIAL_ROBOT = random.randint(0,359)
            DISTANCIA_ACTUAL = calcularDistanciaPuntos(POSICION_INICIAL_ROBOT, POSICION_INICIAL_PELOTA)
            ### EN LA UI MIGUEL
            ### GIRAR AL ROBOT A SU NUEVO ANGULO DE GIRO
            rotarJugador(playerA, DIRECCION_INICIAL_ROBOT)
            ### SIMULAR EL MOVIMIENTO DEL ROBOT A SU NUEVO SITIO
            moverJugador(playerX, playerY, POSICION_INICIAL_ROBOT[0], POSICION_INICIAL_ROBOT[1])
            ### SIMULAR EL MOVIMIENTO DE LA PELOTA A SU NUVEO SITIO
            moverPelota(ballX, ballY, POSICION_INICIAL_PELOTA[0], POSICION_INICIAL_PELOTA[1])


print("FIN SIMULACION")