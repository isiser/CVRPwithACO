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

        # kreiranje liste mrava
        self.ant = [Ant(len(distanceMatrix), self.vehicleCapacities,
                        self.numberOfVehicles) for i in range(numberOfAnts)]

        # statistika
        self.bestAnt = Ant(len(distanceMatrix),
                           self.vehicleCapacities, self.numberOfVehicles)

        self.bestIterationAnt = Ant(len(distanceMatrix),
                                    self.vehicleCapacities, self.numberOfVehicles)

        self.everyIterationResult = []

        # MMAS
        self.trail_max = -1
        self.trail_min = -1

    def findNearestNeighbourList(self):
        listOfNeighbour = []

        for i in range(self.numberOfCities):
            listOfNeighbour.append(np.argsort(self.distanceMatrix[i]))
        return listOfNeighbour

    def NearestNeighbourHeuristic(self):
        visited_cities = set()
        current_city = 0
        visited_cities.add(0)
        distance_passed = 0
        i = 0
        while i < self.numberOfCities:
            for city in self.findNearestNeighbourList()[current_city]:
                if city not in visited_cities:
                    distance_passed = distance_passed + \
                        self.distanceMatrix[current_city, city]
                    current_city = city
                    visited_cities.add(current_city)
                    break
            i = i + 1
        return distance_passed

    def InitializePheromoneTrail(self):
        self.trail_max = 1 / (self.rho * self.NearestNeighbourHeuristic())
        self.trail_min = self.trail_max / (2 * self.numberOfCities)

        self.pheromoneMatrix = np.full(
            (self.numberOfCities, self.numberOfCities), self.trail_max)

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
        #print("najbolji mravj unutraaaa 1: " + str(self.bestAnt.tour_length))
        tour = self.ant[k].tour
        numOfVisitedNodes = len(tour)
        tour_length = 0.0
        for i in range(numOfVisitedNodes-1):
            indexSource = tour[i]
            indexDestination = tour[i+1]
            tour_length = tour_length + \
                self.distanceMatrix[indexSource][indexDestination]
        #print("najbolji mravj unutraaaa 2: " + str(self.bestAnt.tour_length))
        return tour_length

    def ComputeVehicleTourLength(self, k, i):
        tour = self.ant[k].vehicle[i].vehicleTour
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
        self.ant[k].vehicleInUse.vehicleTour.append(self.ant[k].tour[0])
        # uzeti iduce slobodno vozilo iz liste
        index = next((i for i, item in enumerate(self.ant[k].vehicle)
                     if item.usedVehicle == False), -1)
        if index != -1:
            self.ant[k].vehicleInUse = self.ant[k].vehicle[index]
            self.ant[k].vehicleInUse.usedVehicle = True
            self.ant[k].vehicleInUse.vehicleTour.append(0)
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
            if(self.ant[k].antLoad + self.demands[j-1] > self.ant[k].vehicleInUse.capacity):
                self.ReturnToDepot(k)
            else:
                self.ant[k].tour.append(j)
                self.ant[k].visited[j] = True
                self.ant[k].antLoad = self.ant[k].antLoad + self.demands[j-1]

                self.ant[k].vehicleInUse.vehicleTour.append(j)

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
                self.ant[k].vehicle[v].vehicleTour.clear()
                self.ant[k].vehicle[v].usedVehicle = False
        # Postaviti mrave u depot
        for k in range(self.numberOfAnts):
            self.ant[k].tour.append(self.depot)
            self.ant[k].visited[self.depot] = True
        # konstrukcija kompletnog puta
        for k in range(self.numberOfAnts):
            #print("najbolji mravj jeee1: " + str(self.bestAnt.tour_length))
            # Postaviti prvo vozilo za korištenje
            self.ant[k].vehicleInUse = self.ant[k].vehicle[0]
            self.ant[k].vehicleInUse.usedVehicle = True
            self.ant[k].vehicleInUse.vehicleTour.append(0)
            step = 0
            while self.UnvisitedNodes(k) > 0:
                step = step + 1
                # pravilo kako će svaki mrav odabrati svoj put
                self.DecisionRule(k, step)
            #print("najbolji mravj jeee2: " + str(self.bestAnt.tour_length))
            # mrav se vraća u početni grad i računa se duljina puta mrava
            self.ant[k].tour.append(self.ant[k].tour[0])
            #print("najbolji mravj jeee3: " + str(self.bestAnt.tour_length))
            self.ant[k].tour_length = self.ComputeTourLength(k)

            #print("najbolji mravj jeee4: " + str(self.bestAnt.tour_length))
            # vozilo se vraća u početni grad i računa se duljina puta svakog vozila
            self.ant[k].vehicleInUse.vehicleTour.append(self.ant[k].tour[0])
            for i in range(self.numberOfVehicles):
                self.ant[k].vehicle[i].vehicleTourLength = self.ComputeVehicleTourLength(
                    k, i)
            #print("najbolji mravj jeee5: " + str(self.bestAnt.tour_length))

    def BestSolutionSoFar(self):
        self.ant.sort(key=operator.attrgetter('tour_length'))

    def UpdateMinAndMax(self):
        #self.trail_max = 1 / ((1 - self.rho) * self.bestAnt.tour_length)
        self.trail_max = 1 / (self.rho * self.bestAnt.tour_length)
        self.trail_min = self.trail_max * \
            (1 - (0.05 ** (1 / self.numberOfCities))) / \
            (((self.numberOfCities / 2) - 1) * (0.05 ** (1 / self.numberOfCities)))

    def UpdateStatics(self):
        # ispis tura svakog mrava i njegovog vozila
        for i in range(self.numberOfAnts):
            print("Mrav [" + str(i+1) + "]: Tura " + str(self.ant[i].tour) +
                  ", prijedeni put: " + str(self.ant[i].tour_length))
            for j in range(self.numberOfVehicles):
                print("\tVozilo [" + str(j+1) + "]: Tura " + str(self.ant[i].vehicle[j].vehicleTour) +
                      ", prijedeni put: " + str(self.ant[i].vehicle[j].vehicleTourLength))

        # pronalazak najboljeg rjesenja trenutne iteracije
        self.BestSolutionSoFar()
        self.bestIterationAnt.tour = self.ant[0].tour
        self.bestIterationAnt.tour_length = self.ant[0].tour_length
        print("\n")
        print("Najbolje rješenje ove iteracije: " + str(self.bestIterationAnt.tour) +
              "--> " + str(self.bestIterationAnt.tour_length))

        self.everyIterationResult.append(
            round(self.bestIterationAnt.tour_length))

        # pronalazak najboljeg globalnog rjesenja
        if self.bestIterationAnt.tour_length < self.bestAnt.tour_length or self.bestAnt.tour_length == -1:
            self.bestAnt.tour_length = self.bestIterationAnt.tour_length
            self.bestAnt.tour = self.bestIterationAnt.tour
        print("Najbolje rješenje do sada: " + str(self.bestAnt.tour) +
              "--> " + str(self.bestAnt.tour_length))

        # azuriranje min and max
        self.UpdateMinAndMax()

    def Evaporate(self):
        self.pheromoneMatrix = (1 - self.rho) * self.pheromoneMatrix

    # k - redni broj mrava
    def DepositPheromone(self, k):
        delta_tau = 1 / self.ant[k].tour_length

        numberOfVisitedNodes = len(self.ant[k].tour)

        for i in range(numberOfVisitedNodes - 1):
            j = self.ant[k].tour[i]
            l = self.ant[k].tour[i + 1]
            self.pheromoneMatrix[j][l] = self.pheromoneMatrix[j][l] + delta_tau
            self.pheromoneMatrix[l][j] = self.pheromoneMatrix[j][l]

    # Min max update Pheromone trail
    def MMAS_DepositPheromone(self):
        delta_tau = 1 / self.bestAnt.tour_length

        numberOfVisitedNodes = len(self.bestIterationAnt.tour)

        for i in range(numberOfVisitedNodes - 1):
            j = self.bestIterationAnt.tour[i]
            l = self.bestIterationAnt.tour[i + 1]
            self.pheromoneMatrix[j][l] = self.pheromoneMatrix[j][l] + delta_tau
            self.pheromoneMatrix[l][j] = self.pheromoneMatrix[j][l]

    def CheckPhermoneLimits(self):
        for i in range(self.numberOfCities):
            for j in range(i):
                if self.pheromoneMatrix[i][j] < self.trail_min:
                    self.pheromoneMatrix[i][j] == self.trail_min
                    self.pheromoneMatrix[j][i] == self.trail_min
                elif self.pheromoneMatrix[i][j] > self.trail_max:
                    self.pheromoneMatrix[i][j] == self.trail_max
                    self.pheromoneMatrix[j][i] == self.trail_max
        print("MIN: " + str(self.trail_min) + " MAX: " + str(self.trail_max))

    def ASPheromoneUpdate(self):
        self.Evaporate()
        # for k in range(self.numberOfAnts):
        # self.DepositPheromone(k)
        self.MMAS_DepositPheromone()
        self.CheckPhermoneLimits()
        self.PrintNewPheromoneMatrix()

    def PrintNewPheromoneMatrix(self):
        print("------------------NEW PHEROMONE MATRIX------------------")
        print(self.pheromoneMatrix)
