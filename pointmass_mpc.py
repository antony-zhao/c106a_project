import casadi as ca
import numpy as np
import csv
from tqdm import tqdm

def f(x, u):
    A = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0]
    ])
    B = np.array([
        [0.0, 0.0], 
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 0.5]
    ])
    return A @ x + B @ u

class MPC:
    def __init__(
            self,
            N, Ts, f, xs, ys, rs
        ) -> None:

        n, m = 4, 2

        self.opti = ca.Opti()
        self.X = self.opti.variable(n, N+1) # first state plays no role
        self.U = self.opti.variable(m, N+1) # final input plays no role
        self.x0 = self.opti.parameter(n, 1)
        self.r = self.opti.parameter(n, 1) # reference parameter

        Q = np.diag([0,0,0.,0.]) * 5
        R = np.diag([1,1.]) * 0.1

        # box
        for k in range(N+1):
            self.opti.subject_to(self.X[:,k] < np.array([3,3,3,3]))
            self.opti.subject_to(self.X[:,k] > np.array([-3,-3,-3,-3]))
            self.opti.subject_to(self.U[:,k] < np.array([1,1]))
            self.opti.subject_to(self.U[:,k] > np.array([-1,-1]))

        # dynamics
        for k in range(N):
            self.opti.subject_to(self.X[:,k+1] == self.X[:,k] + f(self.X[:,k], self.U[:,k]) * Ts)
        
        # initial conditions
        self.opti.subject_to(self.X[:,0] == self.x0)

        # cylinder constraint
        for k in range(N-1):
            # apply the constraint from 2 timesteps in the future as the quad has relative degree 2
            # to ensure it is always feasible!
            current_time = ca.sum1(k * Ts)
            multiplier = 1 + current_time * 0.1
            for xc, yc, rc in zip(xs, ys, rs):
                self.opti.subject_to(rc ** 2 * multiplier <= (self.X[0,k+2] - xc)**2 + (self.X[1,k+2] - yc)**2)

        # equality constraint to reach end
        self.opti.subject_to(self.X[:,-1] == self.r)

        self.opti.solver('ipopt', {'ipopt.print_level':0, 'print_time':0, 'ipopt.tol': 1e-6})
        cost = ca.MX(0)
        for k in range(N+1):
            x_err = self.r - self.X[:,k]
            cost += x_err.T @ Q @ x_err + self.U[:,k].T @ R @ self.U[:,k]
        self.opti.minimize(cost)

        # test solve
        self.x_sol, self.u_sol = np.zeros([n,N+1]), np.zeros([m,N+1])
        self.opti.set_value(self.x0, np.array([2, 2, 0, 0]))
        self.opti.set_value(self.r, np.array([-2, -2, 0, 0]))
        self.opti.set_initial(self.X, self.x_sol)
        self.opti.set_initial(self.U, self.u_sol)
        sol = self.opti.solve()
        self.x_sol, self.u_sol = sol.value(self.X), sol.value(self.U)

    def __call__(self, x, r):

        self.opti.set_value(self.x0, x)
        self.opti.set_value(self.r, r)
        self.opti.set_initial(self.X, self.x_sol)
        self.opti.set_initial(self.U, self.u_sol)

        sol = self.opti.solve()
        self.x_sol, self.u_sol = sol.value(self.X), sol.value(self.U)
        return self.u_sol[:,0], self.x_sol
    
if __name__ == "__main__":

    from tqdm import tqdm
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    Ti, Ts, Tf = 0.0, 0.1, 15.0
    N = 75
    x = np.array([2.0, 0.0, 0, 0.])
    x_goal = x.copy()
    r = np.array([0, 0, 0, 0])
    xs, ys, rs = [1.0,], [0,], [0.5]

    mpc = MPC(N, Ts, f, xs, ys, rs)
    
    x_hist = [x]
    x_preds = []
    times = np.arange(Ti, Tf, Ts)
    for t in tqdm(times):
        u, preds = mpc(x, r)
        # print(u_preds)
        x_preds.append(preds)
        x += f(x, u) * Ts
        print(x)
        x_hist.append(np.copy(x))

    x_hist = np.vstack(x_hist)
    x_preds = np.stack(x_preds)
    fig, ax = plt.subplots()
    ax.plot(x_hist[1:,0], x_hist[1:,1])
    for xc, yc, rc in zip(xs, ys, rs):
        ax.add_patch(Circle([xc, yc], rc))
    plt.savefig('test.png')
    with open("waypoints.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(x_hist[30:,:][::-10])
        writer.writerow(x_goal)