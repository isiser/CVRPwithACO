import numpy as np
import random


class Ant:

    def __init__(self, n):
        self.n = n  # broj gradova
        self.tour_length = -1  # duljina ture
        # memorija (djelomican/konacan put mrava)
        self.tour = np.full(n + 1, -1)
        # posjeceni gradovi (u pocetku sve False)
        self.visited = np.full(n, False)

    def emptyAntMemory(self):
        for i in range(self.n):
            self.visited[i] = False
        for j in range(self.numberOfVehicles):
            self.usedVehicles[j] = False
        self.antLoad = 0

    def placeAntInDepot(self, step, depot):
        self.tour[step] = depot
        self.visited[depot] = True
