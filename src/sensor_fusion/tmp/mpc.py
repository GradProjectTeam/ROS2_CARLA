import cvxpy as cp

class MPCPlanner:
    def __init__(self, horizon=10, dt=0.1, L=2.5):
        self.horizon = horizon
        self.dt = dt
        self.L = L

    def plan(self, initial_state, ref_path):
        x = cp.Variable(self.horizon)
        y = cp.Variable(self.horizon)
        yaw = cp.Variable(self.horizon)
        v = cp.Variable(self.horizon)
        delta = cp.Variable(self.horizon - 1)
        a = cp.Variable(self.horizon - 1)

        cost = 0
        constraints = []

        # Initial state
        x0, y0, yaw0, v0 = initial_state
        constraints += [x[0] == x0, y[0] == y0, yaw[0] == yaw0, v[0] == v0]

        for t in range(self.horizon - 1):
            cost += cp.square(x[t] - ref_path[t][0]) + cp.square(y[t] - ref_path[t][1])
            cost += cp.square(a[t]) + cp.square(delta[t])
            cost += cp.square(v[t] - 5.0)  # prefer constant speed

            # Kinematic model constraints
            constraints += [
                x[t + 1] == x[t] + v[t] * cp.cos(yaw[t]) * self.dt,
                y[t + 1] == y[t] + v[t] * cp.sin(yaw[t]) * self.dt,
                yaw[t + 1] == yaw[t] + v[t] / self.L * delta[t] * self.dt,
                v[t + 1] == v[t] + a[t] * self.dt,
                cp.abs(delta[t]) <= 0.5,
                cp.abs(a[t]) <= 1.0
            ]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        if prob.status != cp.OPTIMAL:
            print("MPC failed to find optimal trajectory.")
            return None

        return [(x.value[t], y.value[t], yaw.value[t], v.value[t]) for t in range(self.horizon)]
