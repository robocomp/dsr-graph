from math import *

import matplotlib.pyplot as plt

plt.plasma()
import numpy as np
import GaussianMix as GM
import checkboundaries as ck
from normal import Normal
from scipy.spatial import ConvexHull
from rich.console import Console
from math import *

console = Console(highlight=False)
from mpl_toolkits.mplot3d import Axes3D

plt.figure()


# plt.ion()


class Person:
    def __init__(self, x=0, y=0, th=0):
        self.x = x / 1000
        self.y = y / 1000
        self.th = th
        self._radius = 0.30

    def draw(self, sigma_h, sigma_r, sigma_s, rot, draw_personal_space=False):
        # define grid.
        npts = 50
        x = np.linspace(self.x - 4, self.x + 4, npts)
        y = np.linspace(self.y - 4, self.y + 4, npts)

        X, Y = np.meshgrid(x, y)
        Z = self._calculate_personal_space(X, Y, sigma_h, sigma_r, sigma_s, rot)

        if draw_personal_space:
            plt.clf()
            plt.contour(X, Y, Z, 10)
            plt.axis('equal')
            # Body
            body = plt.Circle((self.x, self.y), radius=self._radius, fill=False)
            plt.gca().add_patch(body)

            # Orientation
            x_aux = self.x + self._radius * cos(pi / 2 - self.th)
            y_aux = self.y + self._radius * sin(pi / 2 - self.th)

            # print(y_aux)
            heading = plt.Line2D((self.x, x_aux), (self.y, y_aux), lw=1, color='k')
            plt.gca().add_line(heading)

            fig = plt.figure()

            ax = fig.add_subplot(111, projection='3d')
            # ax = fig.gca(projection='3d')
            ax.plot_surface(X, Y, Z, cmap='plasma')
            plt.pause(0.00001)

            # plt.axis('equal')

        return Z

    def _calculate_personal_space(self, x, y, sigma_h, sigma_r, sigma_s, rot):
        alpha = np.arctan2(y - self.y, x - self.x) - rot - pi / 2
        nalpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        sigma = np.copy(nalpha)
        for i in range(nalpha.shape[0]):
            for j in range(nalpha.shape[1]):
                sigma[i, j] = sigma_r if nalpha[i, j] <= 0 else sigma_h

        a = cos(rot) ** 2 / 2 * sigma ** 2 + sin(rot) ** 2 / 2 * sigma_s ** 2
        b = sin(2 * rot) / 4 * sigma ** 2 - sin(2 * rot) / 4 * sigma_s ** 2
        c = sin(rot) ** 2 / 2 * sigma ** 2 + cos(rot) ** 2 / 2 * sigma_s ** 2

        z = np.exp(-(a * (x - self.x) ** 2 + 2 * b * (x - self.x) * (y - self.y) + c * (y - self.y) ** 2))

        return z


class PersonalSpacesManager:
    def __init__(self):
        self.__personal_spaces = ["intimate", "personal", "social"]
        # sigma_h, sigma_r, sigma_s,  h
        self.__dict_space_param = {"intimate": [2, 1., 1.3, 0.9],
                                   "personal": [2, 1., 1.3, 0.6],  # 0.5
                                   # "social": [3., 1., 1.3, 0.3],  # 0.2
                                   "social": [2, 1., 1.3, 0.3],  # 0.2
                                   }
        ##Limites de la representacion
        self.lx_inf = -50
        self.lx_sup = 50
        self.ly_inf = -50
        self.ly_sup = 50

    def _get_polyline(self, grid, resolution, lx_inf, ly_inf):
        total_points = []
        for j in range(grid.shape[1]):
            for i in range(grid.shape[0]):
                if grid[j, i] > 0:
                    same_cluster, pos = ck.checkboundaries(grid, i, j, total_points)
                    if same_cluster is True:
                        total_points[pos].append([i, j])
                    else:
                        points = [[i, j]]
                        total_points.append(points)

        ret = []
        for list in total_points:
            # los points en el grid sufren una traslacion, con esto los devolvemos a su posicion original
            for points in list:
                points[0] = points[0] * resolution + lx_inf
                points[1] = points[1] * resolution + ly_inf

            points = np.asarray(list)
            hull = ConvexHull(points)
            ret.append(points[hull.vertices])

        return ret

    def get_personal_spaces(self, people_list, represent=False):
        #console.print(' ----- get_personal_spaces -----', style='blue')

        plt.clf()
        dict_spaces = dict(intimate=[], personal=[], social=[])
        dict_spaces_mm = dict(intimate=[], personal=[], social=[])

        for space in self.__personal_spaces:
            normals = []
            for p in people_list:
                person = Person(p.tx, p.ty, p.ry)
                # person.draw(2,1, 4./3.,pi/2 - person.th, drawPersonalSpace=dibujar) #Valores originales
                person.draw(self.__dict_space_param[space][0], self.__dict_space_param[space][1],
                            self.__dict_space_param[space][2], pi / 2 - person.th, draw_personal_space=represent)
                normals.append(Normal(mu=[[person.x], [person.y]],
                                      sigma=[-person.th - pi / 2.,
                                             self.__dict_space_param[space][0],
                                             self.__dict_space_param[space][1],
                                             self.__dict_space_param[space][2]], elliptical=True))

            #print("Number of gaussians ", len(normals))

            resolution = 0.3
            limits = [[self.lx_inf, self.lx_sup], [self.ly_inf, self.ly_sup]]

            _, z = Normal.makeGrid(normals, self.__dict_space_param[space][3], 2, limits=limits, resolution=resolution)
            grid = GM.filterEdges(z, self.__dict_space_param[space][3])


            ordered_points = self._get_polyline(grid, resolution, self.lx_inf, self.ly_inf)

            for pol in ordered_points:
                polyline = []
                polyline_mm = []
                for pnt in pol:
                    polyline.append([pnt[0], pnt[1]])
                    # polyline_mm.append([round(pnt[0] * 1000), round(pnt[1] * 1000)])
                    polyline_mm.append([round(pnt[0] * 1000), round(pnt[1] * 1000)])
                if len(polyline) != 0:
                    dict_spaces[space].append(polyline)
                    dict_spaces_mm[space].append(polyline_mm)

        if represent:
            for soc in dict_spaces["social"]:
                x, y = zip(*soc)
                plt.plot(x, y, color='c', linestyle='None', marker='.')
                plt.pause(0.00001)
            for per in dict_spaces["personal"]:
                x, y = zip(*per)
                plt.plot(x, y, color='m', linestyle='None', marker='.')
                plt.pause(0.00001)

            for inti in dict_spaces["intimate"]:
                x, y = zip(*inti)
                plt.plot(x, y, color='r', linestyle='None', marker='.')
                plt.pause(0.00001)

            plt.axis('equal')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()

        return dict_spaces_mm['intimate'], dict_spaces_mm['personal'], dict_spaces_mm['social']

    ##objects
    def calculate_affordance(self, object):
        shape = object.shape
        if shape == 'trapezoid':
            affordance = self.calculate_affordance_trapezoidal(object)
        elif shape == 'circle':
            affordance = self.calculate_affordance_circular(object)
        elif shape == 'rectangle':
            affordance = self.calculate_affordance_rectangular(object)

        return affordance

    def calculate_affordance_trapezoidal(self, obj):
        left_angle = obj.ry + obj.inter_angle / 2
        right_angle = obj.ry - obj.inter_angle / 2
        

        polyline = [[(obj.tx + obj.width / 2), obj.ty],
                    [(obj.tx - obj.width / 2), obj.ty],
                    
                    [(obj.tx + obj.inter_space * (cos(pi/2 - right_angle))),
                     (obj.ty + obj.inter_space * (sin(pi/2 - right_angle)))],

                    [(obj.tx + obj.inter_space * (cos(pi/2 - left_angle))),
                     (obj.ty + obj.inter_space * (sin(pi/2 - left_angle)))]

                    ]

        return polyline

    def calculate_affordance_circular(self, obj):
        polyline = []
        points = 50
        angle_shift = pi*2 / points
        phi = 0

        for i in range(points):
            phi += angle_shift
            polyline.append([obj.tx + ((obj.width/2 + obj.inter_space)*sin(phi)),
                             obj.ty + ((obj.depth/2 + obj.inter_space)*cos(phi))])


        return polyline

    def calculate_affordance_rectangular(self, obj):

        polyline = [[obj.tx - obj.width / 2 - obj.inter_space, obj.ty - obj.depth / 2 - obj.inter_space],
                    [obj.tx + obj.width / 2 + obj.inter_space, obj.ty - obj.depth / 2 - obj.inter_space],
                    [obj.tx + obj.width / 2 + obj.inter_space, obj.ty + obj.depth / 2 + obj.inter_space],
                    [obj.tx - obj.width / 2 - obj.inter_space, obj.ty + obj.depth / 2 + obj.inter_space]]
        return polyline
