from ant import *


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
        self.ant = [Ant(len(distanceMatrix)) for i in range(numberOfAnts)]

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

    # k je redni broj mrava, a step je svaki korak konstrukcije rješenja
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

            self.ant[k].tour[step] = j
            self.ant[k].visited[j] = True

    def ComputeTourLength(self, k):
        tour = self.ant[k].tour
        tour_length = 0.0

        for i in range(self.numberOfCities):
            tour_length = tour_length + self.distanceMatrix[tour[i]][tour[i+1]]

        return tour_length

    def ConstructSolution(self):
        # pražnjenje memorije mrava
        for k in range(self.numberOfAnts):
            for i in range(self.numberOfCities):
                self.ant[k].visited[i] = False
        step = 0
        # Postaviti mrave u depot
        for k in range(self.numberOfAnts):
            self.ant[k].tour[step] = self.depot
            self.ant[k].visited[self.depot] = True
        # konstrukcija kompletnog puta
        while step < self.numberOfCities:
            step = step + 1
            for k in range(self.numberOfAnts):
                # pravilo kako će svaki mrav odabrati svoj put
                self.DecisionRule(k, step)
        step = self.numberOfCities
        # svaki mrav se vraća u početni grad i računa se duljina puta svakog mrava
        for k in range(self.numberOfAnts):
            self.ant[k].tour[self.numberOfCities] = self.ant[k].tour[0]
            self.ant[k].tour_length = self.ComputeTourLength(k)

    def UpdateStatics(self):
        for i in range(self.numberOfAnts):
            print("Tura mrava -" + str(i) + "- :")
            print(self.ant[i].tour)
            print("Ukupni prijeden put mrava -" + str(i) + "- :")
            print(self.ant[i].tour_length)

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
