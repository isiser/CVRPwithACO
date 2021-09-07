import numpy as np
import csv
import xml.etree.ElementTree as ET
import math
from numpy.lib.function_base import append
import os


class Data:

    def __init__(self, fileName):

        self.fileName = fileName
        self.coordinates = []
        self.demands = []
        self.vehicleNumber = -1
        self.vehicleCapacities = []
        self.numberOfCities = -1

        self.ReadXml()
        self.distanceMatrix = self.CalculateDistanceMatrix()
        self.pheromoneMatrix = self.initilazePheromoneMatrix()

        self.depot = 0

        self.numberOfAnts = 32
        self.alpha = 1
        self.beta = 4
        self.rho = 0.02

    def readDistanceMatrix(self):
        input = np.loadtxt(self.fileName, dtype="i", delimiter=", ")
        return input

    def initilazePheromoneMatrix(self):
        return np.full((self.numberOfCities, self.numberOfCities), 1)

    def SaveToCsv(self, everyIterationResult, numberOfIterations):
        header = ['Broj iteracije', 'Najbolje rjesenje']
        f = open('output.csv', 'w', encoding='UTF8', newline='')
        writer = csv.writer(f)
        writer.writerow(header)
        data = []
        for i in range(numberOfIterations):
            data.append([i, everyIterationResult[i]])
        writer.writerows(data)
        f.close()

    def ReadXml(self):
        current = os.getcwd()
        os.chdir(str(current) + '/inputs')
        tree = ET.parse(self.fileName)
        root = tree.getroot()
        # coordinates
        for child in root.find('network').find('nodes').findall('node'):
            self.coordinates.append(
                [float(child.find('cx').text), float(child.find('cy').text)])
        # number of cities
        self.numberOfCities = len(self.coordinates)
        # demands
        self.demands.append(0)
        for child in root.find('requests').findall('request'):
            request = float(child.find('quantity').text)
            self.demands.append(request)
        # number of vehicles
        a = root.find('info').find('name').text
        b = a.split('-')
        c = b[2]
        if c[1:].startswith(str(0)):
            self.vehicleNumber = int(c[2:])
        else:
            self.vehicleNumber = int(c[1:])
        # fill capacities
        capacity = float(root.find('fleet').find(
            'vehicle_profile').find('capacity').text)
        self.vehicleCapacities = np.full(self.vehicleNumber, capacity)
        print(self.vehicleCapacities)

    def CalculateDistanceMatrix(self):
        distanceMatrix = np.zeros(
            (self.numberOfCities, self.numberOfCities))
        for i in range(self.numberOfCities):
            for j in range(i, self.numberOfCities):
                distanceMatrix[i][j] = math.sqrt((self.coordinates[i][0] - self.coordinates[j][0]) ** 2 + (
                    self.coordinates[i][1] - self.coordinates[j][1]) ** 2)
                distanceMatrix[j][i] = distanceMatrix[i][j]

        return distanceMatrix
