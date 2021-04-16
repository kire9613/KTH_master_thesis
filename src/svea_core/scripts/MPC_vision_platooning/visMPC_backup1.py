#!/usr/bin/env python
from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse

dt = 0.1 # Sampling time

# Matrices
Ad = sparse.csc_matrix([
  [1., 0.],
  [dt, 1.]
])

Bd = sparse.csc_matrix([
  [dt, dt],
  [0., 0.]
])

[nx, nu] = Bd.shape

# Constraints
umin = np.array([-1., -1.])
umax = np.array([0.3, 0.3])

# Objective function
qe = 2.
qb = 2.

R = sparse.diags([qe, qb]) # Engine and braking weight

qv = 2.
qd = 2. 

Q =sparse.diags([qv, qd]) # Relative velocity and distance weights


# Initial and reference states
x0 = np.array([0., 0.5]) # Initial [vrel, d]
xr = np.array([0., 0.3]) # Reference [vrel, d]

# Prediction horizon
N = 10

# Define problem
u = Variable((nu, N))
x = Variable((nx, N+1))
x_init = Parameter(nx)
objective = 0
constraints = [x[:,0] == x_init]
for k in range(N):
    objective += quad_form(x[:,k] - xr, Q) + quad_form(u[:,k], R)
    constraints += [x[:,k+1] == Ad*x[:,k] + Bd*u[:,k]]
    constraints += [umin <= u[:,k], u[:,k] <= umax]

prob = Problem(Minimize(objective), constraints)

# Simulate in closed loop
nsim = 100
for i in range(nsim):
    x_init.value = x0
    prob.solve(solver=OSQP, warm_start=True)
    x0 = Ad.dot(x0) + Bd.dot(u[:,0].value)
    #print('x, '+str(i)+': '+str(x0))
    #print('u, '+str(i)+': '+str(u[:,0].value))
    print(str(u[:,0].value)+str(u[:,1].value)+str(u[:,2].value)+str(u[:,3].value)+str(u[:,4].value)+str(u[:,5].value)+str(u[:,6].value)+str(u[:,7].value)+str(u[:,8].value)+str(u[:,9].value)+)




