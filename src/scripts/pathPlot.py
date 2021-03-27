import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# df = pd.read_csv('../../bin/bfsSearch.txt')
df = pd.read_csv('../../bin/astarsearch.txt')
dfPath = pd.read_csv('../../bin/astarfinalpath.txt');

limit = -1
plt.scatter(-df['Y'][:limit], -df['X'][:limit])
plt.xlim(-1, 17)
plt.xlim(1, -17)
# plt.axis('equal')

plt.plot(-dfPath['Y'], -dfPath['X'], 'r')

plt.grid()