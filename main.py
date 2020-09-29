import sys
from os import path
sys.path.append(path.join(path.dirname(__file__), '.'))

from suspension import *

if __name__ == "__main__":
    # Top A-Arm Points
    frontChassis = PickupPoint(330.55, 97.97, 103.28)
    rearChassis = PickupPoint(319.07, 87.30, -125.34)
    sphericalHolder = PickupPoint(508.58, 95.75, -15.32)

    TOP = PickupLocations(frontChassis, rearChassis, sphericalHolder, 'top')
    TOP.calc_SH(0.01,-15,15)
