#! /usr/bin/env python
import random
import matplotlib.pyplot as plt
import time
import numpy as np
from voronoi import generate_voronoi

def dumpMap(x, y, ax, show=True):
    ax.plot(x, y, 'bx', markersize=8)
    plt.draw()
    if show: plt.pause(0.02)  # Tiempo de espera para poder visualizar el camino tomado, sólo si no se miden tiempos

def best_first_search(intMap, START_POINT, END_POINT):

    # Se inicializa y dibuja el gráfico
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    plt.ion()   # Para mostrar en todo momento el avance

    # Graficar el mapa inicial
    for i in range(intMap.shape[0]):
        for k in range(intMap.shape[1]):
            if intMap[i, k] == 1:
                ax1.plot(k,i,'rs', markersize=10)
                ax2.plot(k,i,'rs', markersize=10)
    # Graficar los puntos de inicio
    ax1.plot(START_POINT['y'], START_POINT['x'], 'ko', markersize=8)
    ax2.plot(START_POINT['y'], START_POINT['x'], 'ko', markersize=8)
    ax1.plot(END_POINT['y'], END_POINT['x'], 'go', markersize=8)
    ax2.plot(END_POINT['y'], END_POINT['x'], 'go', markersize=8)
    plt.draw()
    plt.show()

    # Función para añadir checkpoints en caso de que no encuentre el objetivo
    # Los últimos puntos son los preferenciales al ser una stack LIFO
    # Por lo que es un algoritmo que tiende a la derecha y arriba
    checkPoints = []
    pathPoints = []     # Se guardan todos los puntos obtenidos
    parentPoints = []   # Se guardan los padres de cada punto obtenido 
    def add_checkPoints(x, y):
        # Añade un checkpoints alrededor del punto actual
        # left
        if intMap[x][y - 1] == 0:
            newCheckpoint = (x, y - 1)
            if newCheckpoint not in checkPoints:    # Se asegura de que no se repita en la lista
                checkPoints.append(newCheckpoint)
                pathPoints.append(newCheckpoint)
                parentPoints.append((x, y))
        
        # up
        if intMap[x - 1][y] == 0:
            newCheckpoint = (x - 1, y)
            if newCheckpoint not in checkPoints:
                checkPoints.append(newCheckpoint)
                pathPoints.append(newCheckpoint)
                parentPoints.append((x, y))

        # right
        if intMap[x][y + 1] == 0:
            newCheckpoint = (x, y + 1)
            if newCheckpoint not in checkPoints:
                checkPoints.append(newCheckpoint)
                pathPoints.append(newCheckpoint)
                parentPoints.append((x, y))
        
        # down
        if intMap[x + 1][y] == 0:
            newCheckpoint = (x + 1, y)
            if newCheckpoint not in checkPoints:
                checkPoints.append(newCheckpoint)
                pathPoints.append(newCheckpoint)
                parentPoints.append((x, y))
        
        # Borra el punto actual en caso de que estuviese en los checkpoints
        if (x, y) in checkPoints:
            idx = checkPoints.index((x, y))
            checkPoints.pop(idx)    
        
        return pathPoints
    
    add_checkPoints(START_POINT['x'], START_POINT['y'])

    # Inicialización de parámetros
    nextPoint = {'x': START_POINT['x'], 'y': START_POINT['y']}
    yesExitX, yesExitY = True, True
    directionX, directionY = 0, 0

    # Se ejecuta el bucle hasta encontrar una solución
    done = False
    while not done:

        # Intenta buscar el camino directo hacia la meta
        while yesExitX or yesExitY:
            # Avanzar en X
            if (END_POINT['x'] - nextPoint['x'] != 0):
                directionX = int((END_POINT['x'] - nextPoint['x'])/abs(END_POINT['x'] - nextPoint['x'])) # Determinar la dirección en X
                if intMap[nextPoint['x'] + directionX][nextPoint['y']] == 0:
                    nextPoint['x'] = nextPoint['x'] + directionX
                    intMap[nextPoint['x']][nextPoint['y']] = 2          # Marcar como visitado
                    dumpMap(nextPoint['y'], nextPoint['x'], ax1)
                    add_checkPoints(nextPoint['x'], nextPoint['y'])     # Añadir los checkpoints correspondientes
                    yesExitX = True
                else:
                    yesExitX = False
            else:
                yesExitX = False
                    
            # Verificar si se ha llegado a la meta
            if (END_POINT['x'] == nextPoint['x'] + directionX) and (END_POINT['y'] == nextPoint['y']):
                done = True
                break
            
            # Avanzar en Y
            if (END_POINT['y'] - nextPoint['y'] != 0):
                directionY = int((END_POINT['y'] - nextPoint['y'])/abs(END_POINT['y'] - nextPoint['y'])) # Determinar la dirección en Y
                if intMap[nextPoint['x']][nextPoint['y'] + directionY] == 0:
                    nextPoint['y'] = nextPoint['y'] + directionY
                    intMap[nextPoint['x']][nextPoint['y']] = 2          # Marcar como visitado
                    dumpMap(nextPoint['y'], nextPoint['x'], ax1)
                    add_checkPoints(nextPoint['x'], nextPoint['y'])     # Añadir los checkpoints correspondientes
                    
                    yesExitY = True
                else:
                    yesExitY = False   
            else:
                yesExitY = False

            # Verificar si se ha llegado a la meta
            if (END_POINT['x'] == nextPoint['x']) and (END_POINT['y'] == nextPoint['y'] + directionY):
                done = True
                break

        if done:
            break

        # Cuando no puede dirigirse hacia el objetivo cogemos el último checkpoint
        lastCheckpoint = checkPoints.pop()
        nextPoint['x'], nextPoint['y'] = lastCheckpoint[0], lastCheckpoint[1]
        intMap[nextPoint['x']][nextPoint['y']] = 2          # Marcar como visitado
        dumpMap(nextPoint['y'], nextPoint['x'], ax1)
        add_checkPoints(nextPoint['x'], nextPoint['y'])     # Añadir los checkpoints correspondientes
        yesExitX, yesExitY = True, True
    
    # Gráfica el camino más óptimo encontrado
    lastParent = (nextPoint['x'], nextPoint['y'])
    while lastParent != (START_POINT['x'], START_POINT['y']):
        ax1.plot(lastParent[1], lastParent[0], 'rx', markersize=8)
        plt.draw()
        idx = pathPoints.index(lastParent)
        lastParent = parentPoints[idx]

''' PRESCINDIBLE
# Algoritm Breath First Search proporcionado por el profesor para comparación
def breath_first_search(intMap, START_POINT, END_POINT, show):
    class Node:
        def __init__(self, x, y, myId, parentId):
            self.x = x
            self.y = y
            self.myId = myId
            self.parentId = parentId

    nodes = []

    init = Node(START_POINT['x'], START_POINT['y'], 0, -2)
    nodes.append(init)

    done = False
    goalParentId = -1

    while not done:
        for node in nodes:
            # up
            tmpX = node.x - 1
            tmpY = node.y
            if( intMap[tmpX][tmpY] == 4 ):
                goalParentId = node.myId
                done = True
                break
            elif ( intMap[tmpX][tmpY] == 0 ):
                newNode = Node(tmpX, tmpY, len(nodes), node.myId)
                intMap[tmpX][tmpY] = 2
                dumpMap(tmpY, tmpX, ax2, show=show)
                nodes.append(newNode)

            # down
            tmpX = node.x + 1
            tmpY = node.y
            if( intMap[tmpX][tmpY] == 4 ):
                goalParentId = node.myId
                done = True
                break
            elif ( intMap[tmpX][tmpY] == 0 ):
                newNode = Node(tmpX, tmpY, len(nodes), node.myId)
                intMap[tmpX][tmpY] = 2
                dumpMap(tmpY, tmpX, ax2, show=show)
                nodes.append(newNode)

            # right
            tmpX = node.x
            tmpY = node.y + 1
            if( intMap[tmpX][tmpY] == 4 ):
                goalParentId = node.myId
                done = True
                break
            elif ( intMap[tmpX][tmpY] == 0 ):
                newNode = Node(tmpX, tmpY, len(nodes), node.myId)
                intMap[tmpX][tmpY] = 2
                dumpMap(tmpY, tmpX, ax2, show=show)
                nodes.append(newNode)

            # left
            tmpX = node.x
            tmpY = node.y - 1
            if( intMap[tmpX][tmpY] == 4 ):
                goalParentId = node.myId
                done = True
                break
            elif ( intMap[tmpX][tmpY] == 0 ):
                newNode = Node(tmpX, tmpY, len(nodes), node.myId)
                intMap[tmpX][tmpY] = 2
                dumpMap(tmpY, tmpX, ax2, show=show)
                nodes.append(newNode)

    ok = False
    while not ok:
        for node in nodes:
            if node.myId == goalParentId:
                ax2.plot(node.y, node.x, 'rx', markersize=8)
                plt.draw()
                goalParentId = node.parentId
                if goalParentId == -2:
                    ok = True
'''

if __name__ == "__main__":
    img = np.genfromtxt("Generated/binary_map.csv", delimiter=",")
    skel = generate_voronoi(img)
    start = {"x":1, "y":1}
    end = {"x":8, "y":1}
    skel[2][1] = 0.
    skel[7][1] = 0.
    best_first_search(skel, start, end)
