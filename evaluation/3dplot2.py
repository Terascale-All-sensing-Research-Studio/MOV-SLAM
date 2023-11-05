from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)
R = np.sqrt(X**2 + Y**2)
Z = np.sin(R)
my_col = cm.jet(Z/np.amax(Z))

surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, facecolors = my_col,
        linewidth=0, antialiased=False)
ax.set_zlim(-1.01, 1.01)

plt.show()
