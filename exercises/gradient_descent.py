# gradient descent example from class

import matplotlib.pyplot as plt
from math import sqrt,log

# function parameters
x_0 = [1,1]
kappa = [1,10,100,1000]
c = 1/2
tau = 1/2
epsilon = 1/1000

def func(x,k):
    return x[0]**2 - x[0]*x[1] + k*x[1]**2
def df_dx(x):
    return 2*x[0] - x[1]
def df_dy(x,k):
    return -x[0] + 2*k*x[1]
def neg_gradient(x,k):
    return [-df_dx(x),-df_dy(x,k)]
def norm_sq(p):
    return p[0]**2 + p[1]**2

for k in kappa:
    f_data = []
    x_i = x_0
    p = neg_gradient(x_i,k)
    while not sqrt(norm_sq(neg_gradient(x_i,k))) < epsilon:
        alpha = 1 # step size
        while not func([x_i[0] + alpha*p[0],x_i[1] + alpha*p[1]],k) < func(x_i,k) - c*alpha*norm_sq(p):
            alpha = tau*alpha
        f_data += [func(x_i,k)]
        x_i = [x_i[0]+alpha*p[0],x_i[1]+alpha*p[1]]
        p = neg_gradient(x_i,k)
        # print(x_i,f_data[-1])
    # plt.plot(f_data)
    log_data = [log(f) for f in f_data]
    plt.plot(log_data)
    plt.show()
