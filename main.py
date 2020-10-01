import sys
from os import path
sys.path.append(path.join(path.dirname(__file__), '.'))

from suspension import *

if __name__ == "__main__":
    # Top A-Arm Points
    topFrontChassis = PickupPoint(330.55, 97.97, 103.28)
    topRearChassis = PickupPoint(319.07, 87.30, -125.34)
    topSphericalHolder = PickupPoint(508.58, 95.75, -15.32)

    topArm = PickupLocations(topFrontChassis, topRearChassis, topSphericalHolder, 'top')
    topArm.set_sphericalHolderCoords(0.01,-15,15)

    # Bottom A-Arm Points
    bottomFrontChassis = PickupPoint(268.35, -77.56, 106.71)
    bottomRearChassis = PickupPoint(282.91, -78.69, -126.44)
    bottomSphericalHolder = PickupPoint(538.71, -98.58, 0.00)

    bottomArm = PickupLocations(bottomFrontChassis, bottomRearChassis, bottomSphericalHolder, 'bottom')
    bottomArm.set_sphericalHolderCoords(0.01,-60,60)

    # Debugging
    susp = SuspensionKinematics(topArm, bottomArm)
    susp.set_sphericalPairs()