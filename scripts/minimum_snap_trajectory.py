import numpy as np
from piecewise_polynomial import PiecewisePolynomial
from scipy.linalg import block_diag
from scipy.optimize import minimize

def MinimumSnapTrajectory(x, timePoints, dt, degree, c):
    # creating the polynomial
    k = len(x) - 1 # number of polynoms required
    polynomials = list()
    for i in range(k):
        T = timePoints[i+1] - timePoints[i]
        polynom = PiecewisePolynomial(degree, T, c)
        if i == 0:
            polynom.set_initial_conditions(np.array([x[i], 0, 0]))
        elif i == k-1:
            polynom.set_finial_conditions(np.array([x[k], 0, 0]))
        polynomials.append(polynom)

    # creating matrices
    # Q: hessian
    # A_T finial condition matrix
    # A_0 initial condition matrix
    Q = np.array([])
    A_T = np.zeros(((k+1)*c, k * degree))
    A_0 = np.zeros((k-1, k * degree))
    b_T = np.zeros((len(A_T),))
    b_0 = np.zeros((len(A_0),))
    for i in range(k):
        Q = block_diag(Q ,polynomials[i].Q)
        A_T[i*c:(i+1)*c, i*degree:(i+1)*degree] = polynomials[i].A_0
        A_T[(i+1)*c:(i+2)*c, i*degree:(i+1)*degree] = -polynomials[i].A_T
        if i > 0:
            A_0[i-1, (i)*degree] = 1
            b_0[i-1] = x[i]
        if i == 0:
            b_T[i*c] = x[i]
        elif i == k - 1:
            b_T[(i+1)*c] = -x[-1] 
    Q = np.delete(Q, 0, 0)

    Aeq = np.concatenate((A_0, A_T))
    beq = np.concatenate((b_0, b_T))

    # objective function
    # min pT Q p
    #   s.t. Aeq P - beq = 0
    def objective(p):
        return np.dot(p.T, np.dot(Q, p))

    def constraint(p):
        return np.dot(Aeq, p) - beq

    cons = {'type':'eq', 'fun': constraint}
    p0 = np.zeros((k*degree,)) # initial guess
    # solving QP
    sol = minimize(objective, p0, method="SLSQP", constraints=cons)
    p = sol.x
    p = p.reshape(k, degree) # reshaping the solution for to polynomials
    # returning position, velocity, acceleration, jerk
    # at desired sample rate (0.01 s)
    x = np.array([])
    xdot = np.array([])
    xddot = np.array([])
    xdddot = np.array([])
    time = np.array([])
    elapsed_time = 0
    for i in range(k):
        T = timePoints[i+1] - timePoints[i]
        for j in range(int(T/dt)):
            t = dt * j
            position = p[i, 0] + p[i, 1]*t + p[i, 2]*t**2 \
                + p[i, 3]*t**3 + p[i, 4]*t**4 + p[i, 5]*t**5
            velocity = p[i, 1] + 2*p[i, 2]*t \
                + 3*p[i, 3]*t**2 + 4*p[i, 4]*t**3 + 5*p[i, 5]*t**4
            acceleration = 2*p[i, 2] + 6*p[i, 3]*t \
                + 12*p[i, 4]*t**2 + 20*p[i, 5]*t**3
            jerk = 6*p[i, 3] + 24*p[i, 4]*t + 60*p[i, 5]*t**2
            x = np.append(x, position)
            xdot = np.append(xdot, velocity)
            xddot = np.append(xddot, acceleration)
            xdddot = np.append(xdddot, jerk)
            time = np.append(time, elapsed_time + t)

        elapsed_time += T

    return x, xdot, xddot, xdddot, time