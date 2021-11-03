# closed loop control for inverted pendulum "up" case.

import numpy as np

A = np.array([[0,1],[1,0]])
B = np.array([[0],[1]])
K = np.array([4,4]) # test values to make eigenvals negative real
A_tilde = A - B*K
print(np.linalg.eigvals(A_tilde))
# this gives eigenvalues -1 and -3