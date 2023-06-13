import math

class Vector3:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z
    
  # Used for debugging. This method is called when you print an instance  
  def __str__(self):
    return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"
    
  def __add__(self, v):
    return Vector3(self.x + v.x, self.y + v.y, self.z + v.z)
    
  def __sub__(self, v):
    return Vector3(self.x - v.x, self.y - v.y, self.z - v.z)

  def __mul__(self, n):
    return Vector3(self.x * n, self.y * n, self.z * n)
    
  def __div__(self, n):
    return Vector3(self.x / n, self.y / n, self.z / n)
    
  def zero(self):
    self.x = 0
    self.y = 0
    self.z = 0

  def norm(self):
    return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

  def normalize(self):
    mag = self.norm()
    if mag != 0:
      self.x = self.x / mag
      self.y = self.y / mag
      self.z = self.z / mag
    else:
      self.zero()
    
  @staticmethod
  def dot_product(v1, v2):
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z)
    
#v1 = Vector3(1, 3, 5)
#v2 = Vector3(2, 4, 6)
#v3 = v1 + v2
#v4 = v1 - v2
#v5 = v1 * 2
# v6 = v1 / 2
#n = Vector3.dot_product(v1, v2)

#print(v1)
#print(v2)
#print(v3)
#print(v4)
#print(v5)
# print(v6)
#print(n)