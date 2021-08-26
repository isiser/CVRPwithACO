import numpy as np
import random


class Ant:

    def __init__(self, n):
        self.n = n  # broj gradova
        self.tour_length = -1  # duljina ture
        # memorija (djelomican/konacan put mrava)
        self.tour = []
        # posjeceni gradovi (u pocetku sve False)
        self.visited = np.full(n, False)
        # napunjenost mrava
        self.antLoad = 0

    def emptyAntMemory(self):
        for i in range(self.n):
            self.visited[i] = False
        self.tour.clear()
        self.antLoad = 0

    def placeAntInDepot(self, depot):
        self.tour.insert(depot)
        self.visited[depot] = True
