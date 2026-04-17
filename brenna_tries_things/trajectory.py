import numpy as np


from funrobo_kinematics.core.trajectory_generator import MultiAxisTrajectoryGenerator, MultiSegmentTrajectoryGenerator


class QuinticPolynomial():

    def __init__(self, ndof=None):
        self.ndof = ndof

    def solve(self, q0, qf, qd0, qdf, T, qdd0=None, qddf=None):
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)

        if qd0 is None:
            qd0 = np.zeros_like(q0)
        else:
            qd0 = np.asarray(qd0, dtype=float)

        if qdf is None:
            qdf = np.zeros_like(q0)
        else:
            qdf = np.asarray(qdf, dtype=float)

        if qdd0 is None:
            qdd0 = np.zeros_like(q0)
        if qddf is None:
            qddf = np.zeros_like(q0)

        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T**2, T**3, T**4, T**5],
            [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
            [0, 0, 2, 6*T, 12*T**2, 20*T**3],
        ])

        b = np.vstack([q0, qd0, qdd0, qf, qdf, qddf])
        self.coeff = np.linalg.solve(A, b)

    def generate(self, t0=0, tf=0, nsteps=100):
        t = np.linspace(t0, tf, nsteps)
        X = np.zeros((self.ndof, 3, len(t)))

        for i in range(self.ndof):
            a0, a1, a2, a3, a4, a5 = self.coeff[:, i]

            q = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
            qd = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            qdd = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3

            X[i, 0, :] = q
            X[i, 1, :] = qd
            X[i, 2, :] = qdd

        return t, X
    
ndof = 3
method = QuinticPolynomial(ndof=ndof)
mode = "task"

traj = MultiSegmentTrajectoryGenerator(method=method,
                                        mode=mode,
                                        ndof=ndof,
                                        )
via_points = [[0.1, 0, 0.4], [0.2, 0, 0.3]]

traj.solve(via_points, T=2)
X = traj.generate(nsteps_per_segment=3)

print(f"X pos vec, vel vec, acc vec \n {X[0]}")
print(f"Y pos vec, vel vec, acc vec \n {X[1]}")
print(f"Z pos vec, vel vec, acc vec \n {X[2]}")