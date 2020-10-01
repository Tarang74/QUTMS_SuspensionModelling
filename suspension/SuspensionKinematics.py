from math import sqrt

class SuspensionKinematics:
    """
    This class calculates the Camber and Toe Angles of a single double
    wishbone suspension system
    """
    def __init__(self, topAArm, bottomAArm):
        self.topAArm = topAArm
        self.bottomAArm = bottomAArm

        # Calculate Height between Sphericals
        x = 0
        for i in range(len(self.topAArm.get_staticSphericalHolder().get_point())):
            x = x + (self.bottomAArm.get_staticSphericalHolder().get_point()[i] - self.topAArm.get_staticSphericalHolder().get_point()[i])**2
        self.HbSpherical = sqrt(x)
        
    def set_sphericalPairs(self):
        MAGdiff = [0] * len(self.topAArm.get_sphericalHolderCoords())
        for i in range(len(self.bottomAArm.get_sphericalHolderCoords())):
            print(f"Spherical Holder Loop {i+1} of {len(self.bottomAArm.get_sphericalHolderCoords())}")
            
            bot = self.bottomAArm.get_sphericalHolderCoords()[i]

            for j in range(len(self.topAArm.get_sphericalHolderCoords())):
                top = self.topAArm.get_sphericalHolderCoords()[j]
                tempDiff = [top[0]-bot[0], top[1]-bot[1], top[2]-bot[2]]
                tempDiff = [number**2 for number in tempDiff]
                MAGdiff[j] = sqrt(sum(tempDiff))-self.HbSpherical