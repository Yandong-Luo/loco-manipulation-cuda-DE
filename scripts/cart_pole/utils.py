import numpy as np
from scipy import linalg as la

# Simulation parameters
N = 10  # Number of time steps
dT = 0.02  # Time step size
T_sim = 40.0  # Total simulation time
delta_T_dyn = 0.005  # Delta T for propagating dynamics

# System physical parameters
mc = 1.0  # Cart mass
mp = 0.4  # Pole mass
ll = 0.6  # Pole length
g = 9.81  # Gravity constant

# Control and constraint parameters
k1 = 50  # Spring constant 1
k2 = 50  # Spring constant 2
d_left = 0.40  # Left wall position
d_right = 0.35  # Right wall position
d_max = 0.6  # Maximum displacement
u_max = 20.0  # Maximum control input
lam_max = 30.0  # Maximum lambda value

# State space dimensions
dim_x = 4  # State dimension
dim_u = 3  # Control input dimension

# System matrices
A = np.array([[0.0,               0.0, 1.0, 0.0],
              [0.0,               0.0, 0.0, 1.0],
              [0.0,           g*mp/mc, 0.0, 0.0],
              [0.0, g*(mc+mp)/(ll*mc), 0.0, 0.0]])

B = np.array([[      0.0,        0.0,         0.0],
              [      0.0,        0.0,         0.0],
              [     1/mc,        0.0,         0.0],
              [1/(ll*mc),  1/(ll*mp),  -1/(ll*mp)]])

# Discrete-time system matrices
A_d = np.eye(dim_x) + A*dT
B_d = B*dT

# Cost matrices
Q = np.array([[1.0,   0.0, 0.0,  0.0],
              [0.0, 50.0,  0.0,  0.0],
              [0.0,   0.0, 1.0,  0.0],
              [0.0,   0.0, 0.0, 50.0]])

R = np.array([[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]])/10

QN = la.solve_discrete_are(A_d, B_d, Q, R)

# Initial conditions
x_ini = 0.0
theta_ini = 0.0
dx_ini = 0.0
dtheta_ini = 0.0

# State constraints
x_lb = np.array([-d_max, -np.pi/2, -2*d_max/dT, -np.pi/dT])
x_ub = np.array([ d_max,  np.pi/2,  2*d_max/dT,  np.pi/dT])
u_lb = np.array([-u_max])
u_ub = np.array([ u_max])
lam_lb = np.array([0.0, 0.0])
lam_ub = np.array([lam_max, lam_max])

# Contact model matrices
E = np.array([[-1.0,  ll, 0.0, 0.0],
              [ 1.0, -ll, 0.0, 0.0]])
F = np.array([[1/k1, 0.0],
              [0.0, 1/k2]])
H = np.array([[0.0],
              [0.0]])
c = np.array([d_right, d_left])

# GBD solver settings
max_Benders_loop = 5
max_feas_cuts = 45
lambda_th = 5000
ang_th = 15.0/180.0*np.pi
K_opt = 40
K_feas = 20
Lipshitz = 10
z_list = [[0, 0], [0, 1], [1, 0]]