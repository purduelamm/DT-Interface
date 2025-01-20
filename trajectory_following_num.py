import time
import cvxopt
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb

from MPC.uncMPC import uncMPC

def quadprog(H, f, L=None, k=None, Aeq=None, beq=None, lb=None, ub=None):
    """
    Input: Numpy arrays, the format follows MATLAB quadprog function: https://www.mathworks.com/help/optim/ug/quadprog.html
    Output: Numpy array of the solution
    """
    n_var = H.shape[1]

    P = cvxopt.matrix(H, tc='d')
    q = cvxopt.matrix(f, tc='d')

    if L is not None or k is not None:
        assert(k is not None and L is not None)
        if lb is not None:
            L = np.vstack([L, -np.eye(n_var)])
            k = np.vstack([k, -lb])

        if ub is not None:
            L = np.vstack([L, np.eye(n_var)])
            k = np.vstack([k, ub])

        L = cvxopt.matrix(L, tc='d')
        k = cvxopt.matrix(k, tc='d')

    if Aeq is not None or beq is not None:
        assert(Aeq is not None and beq is not None)
        Aeq = cvxopt.matrix(Aeq, tc='d')
        beq = cvxopt.matrix(beq, tc='d')

    sol = cvxopt.solvers.qp(P, q, L, k, Aeq, beq)

    return np.array(sol['x'])

def MPC_pathgen(ax = -0.067, ay = 0.6522, az = 0.9542, Rg: np.array = np.array([[-0.3256, -0.9455, 0],[0.9455, -0.3256, 0],[0, 0, 1]]), save_path: bool = False):
    cvxopt.solvers.options['show_progress'] = False

    # MPC Parameters
    dt_sim = 0.01
    Tmax = 10
    Nmax_sim = int(Tmax / dt_sim)
    t_sim = np.arange(0, Nmax_sim) * dt_sim

    mpcTimeFactor = 10
    dt = dt_sim * mpcTimeFactor
    Nmax = int(Nmax_sim / mpcTimeFactor)
    t = np.arange(0, Nmax) * dt

    n = 6  # Number of joints
    N = 10  # MPC horizon

    # Design reference trajectory
    yy = ay * np.ones(len(t_sim))
    zz = az * np.ones(len(t_sim))

    # f = 0.2
    # yy = ay * np.cos(2 * np.pi * f * t_sim)
    # zz = az * np.sin(2 * np.pi * f * t_sim) + 0.5 #  * np.cos(2 * np.pi * f * t_sim)
    xx = ax * np.ones_like(yy)

    pg = np.vstack((xx, yy, zz))

    if Rg is None:
        Rg = SE3.Eul(0, 0, 0, unit='deg') # 4x4 transformation matrix with 0 translation

    # MPC setup
    q0 = np.zeros(n)

    A = np.eye(n)
    B = dt * np.eye(n)
    Q = 1e8 * np.eye(n)
    Ru = np.eye(n)
    P = Q
    S, M, Qbar, Rbar, _ = uncMPC(N, A, B, Q, Ru, P)
    H = 2 * np.dot(S.T, np.dot(Qbar, S)) + Rbar
    f0 = 2 * np.dot(S.T, np.dot(Qbar, M))

    # robot setup
    m0609 = rtb.robot.Robot.URDF(file_path='/home/hojunlee/Work/Manipulator-Mujoco/crx_10ial_description/urdf/crx_10ial.xacro')

    # Robot-specific Constraints
    u_UB = np.pi / 180 * np.array([120, 120, 180, 180, 180, 180]) # Max speed (degree/s)
    u_LB = -u_UB
    U_UB = np.tile(u_UB, N)
    U_LB = np.tile(u_LB, N)
    q_UB = np.array([3.14159, 3.14159, 4.71239, 3.31613, 3.14159, 3.92699]) # Motion range (rad)
    q_LB = -q_UB
    Q_UB = np.tile(q_UB, N)
    Q_LB = np.tile(q_LB, N)
    G = np.vstack((S, -S, np.eye(S.shape[1]), -np.eye(S.shape[1])))
    W = np.hstack((Q_UB, -Q_LB, U_UB, -U_LB))
    T = np.vstack((-M, M, np.zeros_like(M), np.zeros_like(M)))
    Wtil = (W + np.dot(T, q0)).reshape(-1,1)
    U_UB = U_UB.reshape(-1,1)
    U_LB = U_LB.reshape(-1,1)

    # Running MPC
    q = np.zeros((n, Nmax_sim))
    q[:, 0] = q0
    u = np.zeros((n, Nmax_sim))
    time_record = []
    k = 0

    for i in range(Nmax_sim - 1):
        if (i == 0) or (i % mpcTimeFactor == 0):
            start = time.time()
            Tg = SE3.Trans(pg[:,i]) * Rg

            # Inverse kinematics (replace this with your specific IK function)
            joint_configs = m0609.ik_LM(Tep=Tg, q0=q[:,i], mask=[1,1,1,1,1,1])
            f = np.dot(f0, (q[:, i] - joint_configs[0]))
            Ustar = quadprog(H=H, f=f, L=G, k=Wtil, lb=U_LB, ub=U_UB)
            uu = Ustar[:n]
            time_record.append(time.time() - start)
            k += 1

        u[:, i] = uu.reshape(-1,)
        q[:, i + 1] = q[:, i] + u[:, i] * dt_sim
        
    print(f'max time: {max(time_record)}')
    print(f'min time: {min(time_record)}')

    if save_path:
        np.save('command.npy', q)
        return
    
    else:
        return q
