import numpy as np


def checkboundaries(grid, i, j, total_points):
    index = 0
    same_cluster = False
    neighbors = []

    ##### EN EL ENTORNO TENGO EN CUENTA LOS PIXELES QUE RODEAN AL PUNTO Y ADEMAS LOS QUE RODEAN A ESTOS #####
    #### SINO DA PROBLEMAS PORQUE LOS CONSIDERA DE CLUSTERS DIFERENTES####

    for indicey in range(-2, 3):
        for indicex in range(-2, 3):
            neighbors.append([i + indicex, j + indicey])

    for a in total_points:
        for b in a:
            if (b in neighbors) and (same_cluster is False):
                # print ("b esta en el neighbors")
                same_cluster = True
                index = total_points.index(a)

            if (b in neighbors) and (same_cluster is True):
                aux = total_points.index(a)
                if aux != index:
                    #   print("El neighbors coincide en dos listas")
                    total_points[index].extend(total_points[aux])
                    total_points.pop(aux)
                    break

    ### SI UN PUNTO TIENE ENTORNOS QUE PERTENECEN A DOS LISTAS
    ### CONCATENAMOS LAS DOS LISTAS Y ELIMINAMOS UNA DE ELLAS PORQUE PERTENECEN AL MISMO CLUSTER

    return same_cluster, index
