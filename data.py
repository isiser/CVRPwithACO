import numpy as np


class Data:

    def __init__(self, fileName):
        self.demands = [1, 2, 3, 2, 2, 1]
        self.vehicleCapacities = [3, 3, 4, 3]
        self.vehicleNumber = 4
        self.depot = 0

        self.fileName = fileName

        self.numberOfAnts = 5
        self.alpha = 1
        self.beta = 2
        self.rho = 0.5
        self.distanceMatrix = self.readDistanceMatrix()
        self.numberOfCities = len(self.distanceMatrix)
        self.pheromoneMatrix = self.initilazePheromoneMatrix()

    def readDistanceMatrix(self):
        input = np.loadtxt(self.fileName, dtype="i", delimiter=", ")
        return input

    def initilazePheromoneMatrix(self):
        return np.full((self.numberOfCities, self.numberOfCities), 1)
