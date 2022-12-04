import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from random import randrange
import numpy as np
import math
from math import sqrt
from networkx.classes.function import path_weight



xy = np.array([[1,1],[4,4],[9,9]])

x = np.array([1,4,7,10,12])
y = np.array([1,4,2,7,3])


xx = np.array([1,4,7,10,12])
z  = np.array([1,12,11,9,3])


fig, axs = plt.subplots(2)
axs[0].plot(x,y, 'r')
axs[1].plot(x,z, 'g--')

plt.show()




#plot 3D knitting of XY and XZ paths

#ax2 = plt.axes(projection='3d')
#ax2.plot3D(x, y, z,'gray')


fig2 = plt.figure()
ax2  = plt.axes(projection="3d")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")

ax2.plot3D(x, y, z, color="black")
plt.show()










