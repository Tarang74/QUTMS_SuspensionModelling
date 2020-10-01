class PickupPoint:
    """
    Stores x, y and z coordinates of a suspension pickup point
    """

    def __init__(self, x, y, z, alias='point'):
        self.x = x
        self.y = y
        self.z = z
        self.alias = alias

    def get_point(self):
        return [self.x, self.y, self.z]

    def set_x(self, x):
        self.x = x

    def get_x(self):
        return self.x 

    def set_y(self, y):
        self.y = y

    def get_y(self):
        return self.y

    def set_z(self, z):
        self.z = z

    def get_z(self):
        return self.z

    def set_alias(self, alias):
        self.alias = alias

    def get_alias(self):
        return self.alias