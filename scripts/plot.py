# importing mplot3d toolkits, numpy and matplotlib
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

x0_log = np.load('outfile.npy')
xw_log = np.load('outfile_without_obstacle.npy')

fig = plt.figure()
 
x, y, z = np.indices((10, 10, 10))
cube1 = (x>2) & (x<7) & (y>2) & (y<7) & (z>2) & (z<7) 

colors = np.empty(cube1.shape, dtype=object)
colors[cube1] = 'red'

ax = plt.axes(projection ='3d')

# defining all 3 axes
z = x0_log[2,:]
x = x0_log[0,:]
y = x0_log[1,:]

zw = xw_log[2,:]
xw = xw_log[0,:]
yw = xw_log[1,:]
 
# plotting
ax.scatter(x, y, z, 'green', label = 'with obstacle')
ax.scatter(xw,yw,zw, 'red', label = 'without obstacle')
ax.voxels(cube1, facecolors=colors)
#ax.set_title('3D line plot geeks for geeks')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.legend()
plt.show()
plt.savefig('3d.png', dpi=300)

