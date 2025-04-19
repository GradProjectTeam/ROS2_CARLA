def dwa(current_state, goal, cost_map, dt=0.1, horizon=1.0):
    from math import cos, sin, hypot
    x, y, theta = current_state
    best_traj = []
    min_cost = float('inf')

    for v in np.linspace(0.5, 1.5, 5):
        for omega in np.linspace(-0.5, 0.5, 5):
            traj = []
            tx, ty, ttheta = x, y, theta
            for _ in range(int(horizon / dt)):
                tx += v * cos(ttheta) * dt
                ty += v * sin(ttheta) * dt
                ttheta += omega * dt
                traj.append((tx, ty))

                xi, yi = int(tx), int(ty)
                if 0 <= xi < cost_map.shape[1] and 0 <= yi < cost_map.shape[0]:
                    if cost_map[yi, xi] > 50:
                        break  # hit obstacle

            dist_cost = hypot(goal[0] - tx, goal[1] - ty)
            total_cost = dist_cost + len(traj)  # can be tuned

            if total_cost < min_cost:
                min_cost = total_cost
                best_traj = traj

    return best_traj
