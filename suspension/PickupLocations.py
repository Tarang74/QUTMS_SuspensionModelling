from sympy import *

class PickupLocations:

    def __init__(self, frontChassis, rearChassis, sphericalHolder, position):
        self.frontChassis = frontChassis
        self.rearChassis = rearChassis
        self.sphericalHolder = sphericalHolder

        if position == 'Top' or position == 'Bottom' or position == 'top' or position == 'bottom':
            self.position = position
        else:
            raise Exception("Error: Position must be top or bottom")

    def calculate_spherical_holders(self, interval, min, max):
        
        pass

    def calc_SH(self, interval, min, max):
        self.calculate_spherical_holders(interval, min, max)