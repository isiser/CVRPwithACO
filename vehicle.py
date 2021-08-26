import numpy as np
import random


class Vehicle:

    def __init__(self, capacity):
        # kapacitet vozila
        self.capacity = capacity
        # duljina ture
        self.vehicleTourLength = -1
        # memorija (djelomican/konacan put vozila)
        self.vehicleTour = []
        self.usedVehicle = False

    def emptyVehicleMemory(self):
        self.vehicleTour.clear()
        self.usedVehicle = False
