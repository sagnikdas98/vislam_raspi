import numpy as np 
f = np.eye(66)

f[56:,56:] = np.eye(10)*25
print(f)