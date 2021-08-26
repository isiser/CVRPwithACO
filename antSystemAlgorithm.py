from re import I
from ant import *
from vehicle import *
import sys
import operator


class AntSystemAlgorithm:

    def __init__(self, numberOfAnts, distanceMatrix, pheromoneMatix,
                 alpha, beta, rho, depot, demands, vehicleCapacities):
        self.numberOfAnts = numberOfAnts
        self.numberOfCities = len(distanceMatrix)
        self.distanceMatrix = distanceMatrix
        self.pheromoneMatrix = pheromoneMatix
        self.alpha = alpha
        self.beta = beta
        self.rho = rho

        # Paramteri za rješavanje CVRP
        self.depot = depot
        self.demands = demands
        self.vehicleCapacities = vehicleCapacities
        self.numberOfVehicles = len(vehicleCapacities)
        # kreiranje liste vozila
        self.vehicle = [Vehicle(self.vehicleCapacities[i])
                        for i in range(self.numberOfVehicles)]
        self.vehicleInUse = self.vehicle[0]

        # kreiranje liste mrava
        self.ant = [Ant(len(distanceMatrix)) for i in range(numberOfAnts)]

        # statistika
        self.bestAnt = Ant(self.numberOfCities)

    # current - trenutni grad, choice - za onaj koji se racuna mogucnost
    def CalculateProbability(self, current, choice, k):
        probability = 0.0
        brojnik = self.pheromoneMatrix[current][choice] ** self.alpha * \
            (1 / self.distanceMatrix[current][choice]) ** self.beta
        nazivnik = 0.0

        for i in range(self.numberOfCities):
            if self.ant[k].visited[i] == False:
                nazivnik = nazivnik + self.pheromoneMatrix[current][i] ** self.alpha * \
                    (1 / self.distanceMatrix[current][i]) ** self.beta
        probability = brojnik / nazivnik

        return probability

    def ComputeTourLength(self, k):
        tour = self.ant[k].tour
        numOfVisitedNodes = len(tour)
        tour_length = 0.0
        for i in range(numOfVisitedNodes-1):
            indexSource = tour[i]
            indexDestination = tour[i+1]
            tour_length = tour_length + \
                self.distanceMatrix[indexSource][indexDestination]
        return tour_length

    def ComputeVehicleTourLength(self, i):
        tour = self.vehicle[i].vehicleTour
        numOfVisitedNodes = len(tour)
        tour_length = 0.0
        for i in range(numOfVisitedNodes-1):
            indexSource = tour[i]
            indexDestination = tour[i+1]
            tour_length = tour_length + \
                self.distanceMatrix[indexSource][indexDestination]
        return tour_length

    def ReturnToDepot(self, k):
        self.ant[k].tour.append(self.ant[k].tour[0])
        self.ant[k].antLoad = 0
        self.vehicleInUse.vehicleTour.append(self.ant[k].tour[0])
        # uzeti iduce slobodno vozilo iz liste
        index = next((i for i, item in enumerate(self.vehicle)
                     if item.usedVehicle == False), -1)
        if index != -1:
            self.vehicleInUse = self.vehicle[index]
            self.vehicleInUse.usedVehicle = True
            self.vehicleInUse.vehicleTour.append(0)
        else:
            print("Nema slobodnih vozila!")
            sys.exit()

    # k je redni broj mrava

    def DecisionRule(self, k, step):
        # c = trenutni grad mrava k
        c = self.ant[k].tour[step - 1]
        sum_probabilites = 0.0
        selection_probability = np.zeros(self.numberOfCities)
        for j in range(self.numberOfCities):
            if self.ant[k].visited[j]:
                selection_probability[j] = 0.0
            else:
                selection_probability[j] = self.CalculateProbability(c, j, k)
                sum_probabilites = sum_probabilites + selection_probability[j]
        if sum_probabilites == 0.0:
            return
        else:
            cumulativ = np.cumsum(selection_probability)
            r = np.random.uniform(0, 1)
            j = 0
            p = cumulativ[j]
            while(p <= r):
                j = j + 1
                p = cumulativ[j]
            if(self.ant[k].antLoad + self.demands[j-1] > self.vehicleInUse.capacity):
                self.ReturnToDepot(k)
            else:
                self.ant[k].tour.append(j)
                self.ant[k].visited[j] = True
                self.ant[k].antLoad = self.ant[k].antLoad + self.demands[j-1]

                self.vehicleInUse.vehicleTour.append(j)

    def UnvisitedNodes(self, k):
        return np.size(self.ant[k].visited) - np.count_nonzero(self.ant[k].visited)

    def ConstructSolution(self):
        # pražnjenje memorije mrava
        for k in range(self.numberOfAnts):
            for i in range(self.numberOfCities):
                self.ant[k].visited[i] = False
            self.ant[k].tour.clear()
            self.ant[k].antLoad = 0
        # pražnjenje memorije vozila
        for v in range(self.numberOfVehicles):
            self.vehicle[v].vehicleTour.clear()
            self.vehicle[v].usedVehicle = False
        # Postaviti mrave u depot
        for k in range(self.numberOfAnts):
            self.ant[k].tour.append(self.depot)
            self.ant[k].visited[self.depot] = True
        # konstrukcija kompletnog puta
        for k in range(self.numberOfAnts):
            print("******* Mrav[" + str(k) + "] ********")
            # pražnjenje memorije vozila
            for v in range(self.numberOfVehicles):
                self.vehicle[v].vehicleTour.clear()
                self.vehicle[v].usedVehicle = False
            # Postaviti prvo vozilo za korištenje
            self.vehicleInUse = self.vehicle[0]
            self.vehicleInUse.usedVehicle = True
            self.vehicleInUse.vehicleTour.append(0)
            step = 0
            while self.UnvisitedNodes(k) > 0:
                step = step + 1
                # pravilo kako će svaki mrav odabrati svoj put
                self.DecisionRule(k, step)

            # mrav se vraća u početni grad i računa se duljina puta mrava
            self.ant[k].tour.append(self.ant[k].tour[0])
            self.ant[k].tour_length = self.ComputeTourLength(k)

            # vozilo se vraća u početni grad i računa se duljina puta svakog vozila
            self.vehicleInUse.vehicleTour.append(self.ant[k].tour[0])
            for i in range(self.numberOfVehicles):
                self.vehicle[i].vehicleTourLength = self.ComputeVehicleTourLength(
                    i)

    def BestSolutionSoFar(self):
        self.ant.sort(key=operator.attrgetter('tour_length'))

    def UpdateStatics(self):

        for i in range(self.numberOfAnts):
            print("Mrav [" + str(i+1) + "]: Tura " + str(self.ant[i].tour) +
                  ", prijedeni put: " + str(self.ant[i].tour_length))
            for j in range(self.numberOfVehicles):
                print("\tVozilo [" + str(j+1) + "]: Tura " + str(self.vehicle[j].vehicleTour) +
                      ", prijedeni put: " + str(self.vehicle[j].vehicleTourLength))

        self.BestSolutionSoFar()
        bestIteration = self.ant[0]
        print("\n")
        print("Najbolje rješenje ove iteracije: " + str(bestIteration.tour) +
              "--> " + str(bestIteration.tour_length))

        if bestIteration.tour_length < self.bestAnt.tour_length or self.bestAnt.tour_length == -1:
            self.bestAnt = bestIteration

        print("Najbolje rješenje do sada: " + str(self.bestAnt.tour) +
              "--> " + str(self.bestAnt.tour_length))

    def Evaporate(self):
        self.pheromoneMatrix = self.pheromoneMatrix * self.rho

    # k - redni broj mrava
    def DepositPheromone(self, k):
        delta_tau = 1 / self.ant[k].tour_length
        for i in range(self.numberOfCities):
            j = self.ant[k].tour[i]
            l = self.ant[k].tour[i + 1]
            self.pheromoneMatrix[j][l] = self.pheromoneMatrix[j][l] + delta_tau
            self.pheromoneMatrix[l][j] = self.pheromoneMatrix[j][l]

    def ASPheromoneUpdate(self):
        self.Evaporate()
        for k in range(self.numberOfAnts):
            self.DepositPheromone(k)
        self.PrintNewPheromoneMatrix()

    def PrintNewPheromoneMatrix(self):
        print("------------------NEW PHEROMONE MATRIX------------------")
        print(self.pheromoneMatrix)
