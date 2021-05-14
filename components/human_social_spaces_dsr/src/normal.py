import numpy as np


class Normal(object):
    _dimensions = 0
    _mu = np.empty(0)
    _sigma = np.empty(0)
    _ellip = False

    two_pi = np.pi + np.pi
    half_pi = np.pi * 0.5

    def __init__(self, mu, sigma, dimensions=None, elliptical=False):

        if dimensions is None:
            dimensions = len(mu)

        self._dimensions = dimensions

        if dimensions == len(mu):
            self._mu = np.array(mu)
            self._sigma = np.array(sigma)

            if dimensions == 2:
                self._ellip = elliptical

    def __str__(self):
        return "Normal ( mu = {0} , sigma = {1} , elliptical = {2} )".format(self._mu.tolist(), self._sigma.tolist(),
                                                                             self._ellip)

    def __repr__(self):
        return str(self)

    @staticmethod
    def _calcExp(grids, mu, const, sigmaI, result):
        if len(grids.shape) == 2:
            for i in range(grids.shape[-1]):
                sub = grids[:, i][np.newaxis].T - mu
                result[i] += const * np.exp(-0.5 * np.dot(sub.T, np.dot(sigmaI, sub))[0, 0])
        else:
            for i in range(grids.shape[-1]):
                Normal.calcExp(grids[:, i], mu, const, sigmaI, result[i])

    @staticmethod
    def makeGrid(normals, h, dimensions=None, limits=None, resolution=0.1, grids=None):
        if len(normals) > 0:
            axes = []
            if grids is None:
                if dimensions is None:
                    dimensions = normals[0].getDimensions()
                if limits is None:
                    limits = [[np.inf, -np.inf] for _ in range(dimensions)]
                    for normal in normals:
                        local = normal.getBounds(h)
                        for i in range(dimensions):
                            if local[i][0] < limits[i][0]:
                                limits[i][0] = local[i][0]
                            if local[i][1] > limits[i][1]:
                                limits[i][1] = local[i][1]
                for limit in limits:
                    axes.append(np.arange(limit[0], limit[1], resolution))
                grids = np.array(np.meshgrid(*axes))
            result = np.zeros(grids.shape[1:])
            for normal in normals:
                normal.addTo(grids, result)
            return grids, result

    def move(self):
        self._mu += np.random.uniform(-0.1, 0.1, self._mu.shape)
        self._sigma += np.random.uniform(0.1, 0.11, self._sigma.shape)
        if self._ellip:
            self._sigma[0] = np.arctan2(np.sin(self._sigma[0]), np.cos(self._sigma[0]))
            self._sigma[1] = 2.0
            self._sigma[2] = 1.0
            self._sigma[3] = 4 / 3

    def getBounds(self, h):

        if self._ellip:

            const = 3 / (0.8 + h)

            r = const / self._sigma[1]
            min_x = max_x = r * np.cos(self._sigma[0] + np.pi) + self._mu[0, 0]
            min_y = max_y = r * np.sin(self._sigma[0] + np.pi) + self._mu[1, 0]

            r = const / self._sigma[2]
            x = r * np.cos(self._sigma[0]) + self._mu[0, 0]
            y = r * np.sin(self._sigma[0]) + self._mu[1, 0]

            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y

            r = const / self._sigma[3]
            x = r * np.cos(self._sigma[0] + Normal.half_pi) + self._mu[0, 0]
            y = r * np.sin(self._sigma[0] + Normal.half_pi) + self._mu[1, 0]

            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y

            x = r * np.cos(self._sigma[0] - Normal.half_pi) + self._mu[0, 0]
            y = r * np.sin(self._sigma[0] - Normal.half_pi) + self._mu[1, 0]

            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y

            limits = [[min_x, max_x], [min_y, max_y]]
        else:
            const = 2
            limits = [[np.inf, -np.inf] for _ in range(self._dimensions)]
            for i in range(self._dimensions):
                cov = self._sigma[i, i] * const + const / self._sigma[i, i]
                axis = self._mu[i, 0] - cov
                if axis < limits[i][0]:
                    limits[i][0] = axis
                axis = self._mu[i, 0] + cov
                if axis > limits[i][1]:
                    limits[i][1] = axis
        return limits

    def addTo(self, grids, result):
        if self._ellip:

            dx = grids[0] - self._mu[0, 0]
            dy = grids[1] - self._mu[1, 0]

            alpha = np.arctan2(dy, dx) - (self._sigma[0] - Normal.half_pi)
            alpha = alpha > (Normal.two_pi * np.floor((alpha + np.pi) / Normal.two_pi))
            sigma_sq = np.where(alpha, self._sigma[1] * self._sigma[1], self._sigma[2] * self._sigma[2])

            cos_theta = np.cos(self._sigma[0])
            cos_theta = cos_theta * cos_theta * 0.5
            sin_theta = np.sin(self._sigma[0])
            sin_theta = sin_theta * sin_theta * 0.5
            sin_2theta = np.sin(self._sigma[0] + self._sigma[0]) * 0.25

            sigma_3_sq = self._sigma[3] * self._sigma[3]

            a = cos_theta * sigma_sq + sin_theta * sigma_3_sq
            b = (sin_2theta * (sigma_sq - sigma_3_sq)) * dx * dy
            c = sin_theta * sigma_sq + cos_theta * sigma_3_sq

            result += np.exp(-(a * dx * dx + (b + b) + c * dy * dy))

        elif self._dimensions == 2:
            ox = np.sqrt(self._sigma[0, 0])
            oy = np.sqrt(self._sigma[1, 1])
            rho = self._sigma[0, 1] / (ox * oy)
            rho_sqrt = np.sqrt(1.0 - rho * rho)
            rho_sqrt += rho_sqrt
            dx = (grids[0] - self._mu[0, 0]) / ox
            dy = (grids[1] - self._mu[1, 0]) / oy
            result += (1.0 / (np.pi * ox * oy * rho_sqrt)) * np.exp((-1.0 / rho_sqrt) *
                                                                    ((dx * dx) + (dy * dy) - ((rho + rho) * dx * dy)))
        else:
            Normal._calcExp(grids, self._mu, (1.0 / np.sqrt(((Normal.two_pi) ** self._dimensions) *
                                                            np.linalg.det(self._sigma))), np.linalg.inv(self._sigma),
                            result)

    def getMu(self):
        return self._mu

    def getSigma(self):
        return self._sigma

    def getDimensions(self):
        return self._dimensions
