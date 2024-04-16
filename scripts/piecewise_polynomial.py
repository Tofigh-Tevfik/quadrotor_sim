import numpy as np

# class for each polynomial
class PiecewisePolynomial:
    def __init__(self, degree, T, c):
        self.Q = calc_hessian(T, degree) # calculate the hessian on initialization
        self.A_0, self.A_T = find_initial_final_condition(T, degree, c) # along with A_0, A_T
        # set the initial and final conditions to 0
        # later we can change the with the two methods below
        self.b_T = np.array(len(self.A_T))
        self.b_0 = np.array(len(self.A_0))

    def set_initial_conditions(self, b_0):
        self.b_0 = b_0

    def set_finial_conditions(self, b_T):
        self.b_T = b_T

# c is the degree of continuity (max(c) = 8)
def find_initial_final_condition(T, degree, c):
    # A_0 is the inital condition matrix
    A_0 = np.array([
        [1, 0, 0, 0,  0,   0,   0,    0], # initial position
        [0, 1, 0, 0,  0,   0,   0,    0], # initial velocity
        [0, 0, 2, 0,  0,   0,   0,    0], # initial acceleration
        [0, 0, 0, 6,  0,   0,   0,    0], # initial jerk
        [0, 0, 0, 0, 24,   0,   0,    0], # initial snap
        [0, 0, 0, 0,  0, 120,   0,    0],
        [0, 0, 0, 0,  0,   0, 720,    0],
        [0, 0, 0, 0,  0,   0,   0, 5040]
    ])

    # A_T is the final condition or condition at the second T
    A_T = np.array([
        [1, 1*T, 1*T**2, 1*T**3,  1*T**4,   1*T**5,   1*T**6,     1*T**7], # final position
        [0,   1,    2*T, 3*T**2,  4*T**3,   5*T**4,   6*T**5,     7*T**6], # final velocity
        [0,   0,      2,    6*T, 12*T**2,  20*T**3,  30*T**4,    42*T**5], # final acceleration
        [0,   0,      0,      6,    24*T,  60*T**2, 120*T**3,   210*T**4], # final jerk
        [0,   0,      0,      0,      24,    120*T, 360*T**2,   840*T**3], # final snap
        [0,   0,      0,      0,       0,      120,    720*T,  2520*T**2],
        [0,   0,      0,      0,       0,        0,      720,     5040*T],
        [0,   0,      0,      0,       0,        0,        0,       5040]
    ])

    A_0 = A_0[0:c, 0:degree]
    A_T = A_T[0:c, 0:degree]
    return A_0, A_T

def calc_hessian(T, degree):
    r = 4 # snap is the 4th derivative
    Q = np.zeros((degree, degree))

    for i in range(1, degree+1):
        for l in range(1, degree + 1):
            if i >= r and l >= r:
                Q_r = 1
                for m in range(r):
                    Q_r = Q_r * (i-m)*(l-m)
                Q[i-1, l-1] = 2 * Q_r * T**(i+l-2*r+1)/(i+l-2*r+1)
            else:
                Q[i-1, l-1] = 0

    return Q