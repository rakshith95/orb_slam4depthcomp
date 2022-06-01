import numpy as np 
import cv2
from matplotlib import pyplot as plt 
import pandas as pd
import sys
from mpl_toolkits.mplot3d import Axes3D

traj_path = sys.argv[1]
df = pd.read_csv(traj_path, sep=' ', header=None )
xs =  np.asarray(df.iloc[:, 1])
ys =  np.asarray(df.iloc[:, 2])
zs =  np.asarray(df.iloc[:, 3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs,ys, zs)
plt.show()