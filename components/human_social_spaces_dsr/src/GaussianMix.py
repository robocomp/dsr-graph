# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# def isEdge(z, x, y, h):
#     if z[y, x] >= h:
#         len_x = z.shape[1] - 1
#         len_y = z.shape[0] - 1
#         if x > 0:
#             if z[y, x - 1] < h:
#                 return True
#             if y > 0 and z[y - 1, x - 1] < h:
#                 return True
#             if y < len_y and z[y + 1, x - 1] < h:
#                 return True
#         if x < len_x:
#             if z[y, x + 1] < h:
#                 return True
#             if y > 0 and z[y - 1, x + 1] < h:
#                 return True
#             if y < len_y and z[y + 1, x + 1] < h:
#                 return True
#         if y > 0 and z[y - 1, x] < h:
#             return True
#         if y < len_y and z[y + 1, x] < h:
#             return True
#     return False


# def filterEdges(z, h):
#     result = np.zeros(z.shape, dtype=np.uint8)
#     for y in range(z.shape[0]):
#         for x in range(z.shape[1]):
#             if isEdge(z, x, y, h):
#                 result[y, x] = 1
#             # print("result:")
#             # print(result, y, x)

#     return result

def isEdge(z, h):
    len_x, len_y = z.shape[1] - 1, z.shape[0] - 1
    
    # Calcula las diferencias con los vecinos en cada dirección
    left = np.roll(z, shift=(0, -1), axis=(0, 1)) < h
    right = np.roll(z, shift=(0, 1), axis=(0, 1)) < h
    up = np.roll(z, shift=(-1, 0), axis=(0, 1)) < h
    down = np.roll(z, shift=(1, 0), axis=(0, 1)) < h
    
    # Verifica si algún vecino cumple con la condición
    return (z >= h) & (left | right | up | down)

def filterEdges(z, h):
    edge_mask = isEdge(z, h).astype(np.uint8)
    return edge_mask
