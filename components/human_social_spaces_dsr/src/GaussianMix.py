# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from normal import Normal


def isEdge(z, x, y, h):
    if z[y, x] >= h:
        len_x = z.shape[1] - 1
        len_y = z.shape[0] - 1
        if x > 0:
            if z[y, x - 1] < h:
                return True
            if y > 0 and z[y - 1, x - 1] < h:
                return True
            if y < len_y and z[y + 1, x - 1] < h:
                return True
        if x < len_x:
            if z[y, x + 1] < h:
                return True
            if y > 0 and z[y - 1, x + 1] < h:
                return True
            if y < len_y and z[y + 1, x + 1] < h:
                return True
        if y > 0 and z[y - 1, x] < h:
            return True
        if y < len_y and z[y + 1, x] < h:
            return True
    return False


def filterEdges(z, h):
    result = np.zeros(z.shape, dtype=np.uint8)
    for y in range(z.shape[0]):
        for x in range(z.shape[1]):
            if isEdge(z, x, y, h):
                result[y, x] = 1
            # print("result:")
            # print(result, y, x)

    return result
