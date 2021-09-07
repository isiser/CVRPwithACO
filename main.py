from antSystemAlgorithm import AntSystemAlgorithm
import sys
import numpy as np
from data import *


def main():
    data = Data(sys.argv[1])

    print("---------INITIALIZE DATA-----------------\n")
    antSystemAlgorithm = AntSystemAlgorithm(
        data.numberOfAnts, data.distanceMatrix, data.pheromoneMatrix,
        data.alpha, data.beta, data.rho, data.depot, data.demands, data.vehicleCapacities)

    print("---------PHEROMONE MATRIX-----------------\n")
    antSystemAlgorithm.InitializePheromoneTrail()

    i = 0
    while(i < 1000):
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

    data.SaveToCsv(antSystemAlgorithm.everyIterationResult, i)


if __name__ == "__main__":
    main()
