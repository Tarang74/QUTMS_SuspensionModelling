from sympy import *
import numpy as np
from math import sqrt, ceil, pi, atan, sin, cos
import itertools

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
        A = np.matrix([
            [1,0,0,-(self.rearChassis.get_x()-self.frontChassis.get_x())],
            [0,1,0,-(self.rearChassis.get_y()-self.frontChassis.get_y())],
            [0,0,1,-(self.rearChassis.get_z()-self.frontChassis.get_z())],
            [(self.rearChassis.get_x()-self.frontChassis.get_x()), (self.rearChassis.get_y()-self.frontChassis.get_y()), (self.rearChassis.get_z()-self.frontChassis.get_z()), -1]
        ])

        b = np.transpose(np.matrix([
            [self.frontChassis.get_x(), self.frontChassis.get_y(), self.frontChassis.get_z(), self.sphericalHolder.get_x()*(self.rearChassis.get_x()-self.frontChassis.get_x()) + self.sphericalHolder.get_y()*(self.rearChassis.get_y()-self.frontChassis.get_y()) + self.sphericalHolder.get_z()*(self.rearChassis.get_z()-self.frontChassis.get_z()) ]
        ]))

        sol = np.linalg.solve(A,b)

        pointRotation = [sol[0].tolist()[0][0], sol[1].tolist()[0][0], sol[2].tolist()[0][0]]
        radius = sqrt((pointRotation[0] - self.sphericalHolder.get_x())**2 + (pointRotation[1] - self.sphericalHolder.get_y())**2 + (pointRotation[2] - self.sphericalHolder.get_z())**2)
        
        origin = [self.frontChassis.get_x(), self.frontChassis.get_y(), self.frontChassis.get_z()]
        sphericalOrigin = [self.sphericalHolder.get_x(), self.sphericalHolder.get_y(), self.sphericalHolder.get_z()]

        BasePointOne = np.subtract(pointRotation, origin)
        BasePointTwo = np.subtract(sphericalOrigin, origin)

        # Rotation Angle about Z Axis
        gamma = 2*pi - atan(BasePointOne[1]/BasePointOne[0])

        self.sphericalHolderCoords = []
        
        for i in range(1, int((max-min)/interval + 1)+1):
            theta = (min + (i*interval))*pi/180

            # Translate Point 1 to Origin(T)
            pointOne = BasePointOne
            pointTwo = BasePointTwo
            
            # Rotate Point 2 onto X axis
            Rz = np.matrix([
                [cos(gamma), -sin(gamma), 0],
                [sin(gamma), cos(gamma), 0],
                [0, 0, 1]
            ])
            pointOne = np.dot(Rz, np.array(pointOne)).transpose()
            pointTwo = np.dot(Rz, np.array(pointTwo)).transpose()

            # Rotation Angle about Y Axis
            beta = atan(pointOne[2]/pointOne[0])
            
            Ry = np.matrix([
                [cos(beta), 0, sin(beta)],
                [0, 1, 0],
                [-sin(beta), 0, cos(beta)]
            ])
            pointOne = np.dot(Ry, pointOne)
            pointTwo = np.dot(Ry, pointTwo)

            # Rotate Around X Axis
            Rx = np.matrix([
                [1, 0, 0],
                [0, cos(theta), -sin(theta)],
                [0, sin(theta), cos(theta)]
            ])
            pointOne = np.dot(Rx, pointOne)
            pointTwo = np.dot(Rx, pointTwo)

            # Rotate to Original Orientation
            pointOne = np.dot(Ry**-1, pointOne)
            pointTwo = np.dot(Ry**-1, pointTwo)
            
            pointOne = np.dot(Rz**-1, pointOne)
            pointTwo = np.dot(Rz**-1, pointTwo)

            # Translate to Original Position
            pointOne = np.vstack(np.diag(np.add(pointOne, origin)))
            pointTwo = np.vstack(np.diag(np.add(pointTwo, origin)))
            
            condensedTwo = list(itertools.chain(*pointTwo.tolist()))
            self.sphericalHolderCoords.append(condensedTwo)
            MAG = [self.sphericalHolderCoords[-1][0] - pointRotation[0], self.sphericalHolderCoords[-1][1] - pointRotation[1], self.sphericalHolderCoords[-1][2] - pointRotation[2]]
            MAG = sum([x**2 for x in MAG])**(1/2)
            

    def calc_SH(self, interval, min, max):
        self.calculate_spherical_holders(interval, min, max)