from antSystemAlgorithm import AntSystemAlgorithm
import sys
import numpy as np
from data import *

"""Way to find pheromone matrix for initilaze"""


def findClosestNeighbour(distanceMatrix):
    listOfNeighbour = []
    for i in range(len(distanceMatrix)):
        listOfNeighbour.append(np.argsort(distanceMatrix[i]))
    return listOfNeighbour


def main():
    data = Data(sys.argv[1])

    print("---------DISTANCE MATRIX-----------------")
    print(data.distanceMatrix)

    print("---------PHEROMONE MATRIX-----------------")
    print(data.pheromoneMatrix)

    print("---------INITIALIZE DATA-----------------\n")
    antSystemAlgorithm = AntSystemAlgorithm(
        data.numberOfAnts, data.distanceMatrix, data.pheromoneMatrix,
        data.alpha, data.beta, data.rho, data.depot, data.demands, data.vehicleCapacities)

    i = 0
    while(i < 5):
        print("****************************************************")
        print("************* ITERACIJA " +
              str(i) + " **************************")
        print("****************************************************")
        i = i + 1
        print("\n---------CONSTRUCT SOLUTION-----------------\n")
        antSystemAlgorithm.ConstructSolution()
        print("\n---------UPDATE STATISTICS-----------------\n")
        antSystemAlgorithm.UpdateStatics()
        print("\n---------UPDATE PHEROMONE TRAIL-------------\n")
        antSystemAlgorithm.ASPheromoneUpdate()


if __name__ == "__main__":
    main()
