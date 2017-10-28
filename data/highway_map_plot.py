import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

data = np.loadtxt('highway_map.csv', delimiter=' ', skiprows=0)

fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.set_title("Map")    
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.plot(data[:,0],data[:,1], c='r', label='path')

leg = ax1.legend()

plt.show()


fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.set_title("S Frenet Coordinate")    
ax1.set_xlabel('#')
ax1.set_ylabel('S')
ax1.plot(data[:,2], c='b', label='S')

leg = ax1.legend()

plt.show()