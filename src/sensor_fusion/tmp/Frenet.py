from scipy.interpolate import CubicSpline

class FrenetSmoother:
    def __init__(self, path_points):
        self.path_points = path_points  # From Hybrid A* → [(x, y, θ), ...]

    def smooth(self):
        if len(self.path_points) < 4:
            return self.path_points  # Not enough points to smooth

        x = [p[0] for p in self.path_points]
        y = [p[1] for p in self.path_points]

        # Calculate cumulative arc length
        s = [0]
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            s.append(s[-1] + np.hypot(dx, dy))

        # Fit cubic splines
        sx = CubicSpline(s, x)
        sy = CubicSpline(s, y)

        # Sample the path smoothly
        s_new = np.linspace(0, s[-1], num=100)
        smooth_path = [(sx(si), sy(si)) for si in s_new]
        return smooth_path
