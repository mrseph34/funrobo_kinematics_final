import numpy as np

from funrobo_kinematics.core.trajectory_generator import MultiAxisTrajectoryGenerator, MultiSegmentTrajectoryGenerator




class CubicPolynomial():
    """
    Cubic interpolation with position and velocity boundary constraints.
    """

    def __init__(self, ndof=None):
        """
        Initialize the trajectory generator.
        """
        self.ndof = ndof

    
    def solve(self, q0, qf, qd0, qdf, T):
        """
        Compute cubic polynomial coefficients for each DOF.

        Parameters
        ----------
        q0 : array-like, shape (ndof,)
            Initial positions.
        qf : array-like, shape (ndof,)
            Final positions.
        qd0 : array-like or None, shape (ndof,)
            Initial velocities. If None, assumed zero.
        qdf : array-like or None, shape (ndof,)
            Final velocities. If None, assumed zero.
        T : float
            Total trajectory duration.
        """
        t0, tf = 0, T
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        qd0 = np.zeros_like(q0) if qd0 is None else np.asarray(qd0, dtype=float)
        qdf = np.zeros_like(q0) if qdf is None else np.asarray(qdf, dtype=float)
        
        A = np.array(
                [[1, t0, t0**2, t0**3],
                 [0, 1, 2*t0, 3*t0**2],
                 [1, tf, tf**2, tf**3],
                 [0, 1, 2*tf, 3*tf**2]
                ])

        b = np.vstack([
            q0,
            qd0,
            qf,
            qdf
        ])
        self.coeff = np.linalg.solve(A, b)
        

    def generate(self, t0=0, tf=0, nsteps=100):
        """
        Generate position, velocity, and acceleration trajectories.

        Parameters
        ----------
        t0 : float
            Start time.
        tf : float
            End time.
        nsteps : int
            Number of time samples.
        """
        t = np.linspace(t0, tf, nsteps)
        X = np.zeros((self.ndof, 3, len(t)))
        for i in range(self.ndof): # iterate through all DOFs
            c = self.coeff[:, i]

            q = c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3
            qd = c[1] + 2 * c[2] * t + 3 * c[3] * t**2
            qdd = 2 * c[2] + 6 * c[3] * t

            X[i, 0, :] = q      # position
            X[i, 1, :] = qd     # velocity
            X[i, 2, :] = qdd    # acceleration

        return t, X




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




class TrapezoidalTrajectory():

    def __init__(self, ndof=None, ramp_fraction=0.35):
        self.ndof = ndof
        self.ramp_fraction = ramp_fraction

    def solve(self, q0, qf, qd0=None, qdf=None, T=1):
        self.q0 = np.asarray(q0, dtype=float)
        self.qf = np.asarray(qf, dtype=float)
        self.T = T
        self.t_ramp = self.ramp_fraction * T
        self.t_cruise_end = T - self.t_ramp
        dq = self.qf - self.q0
        t_cruise = T - 2 * self.t_ramp
        self.v_cruise = dq / (self.t_ramp + t_cruise)
        self.accel = self.v_cruise / self.t_ramp

    def generate(self, t0=0, tf=0, nsteps=100):
        t = np.linspace(t0, tf, nsteps)
        X = np.zeros((self.ndof, 3, len(t)))

        for i in range(self.ndof):
            a = self.accel[i]
            vc = self.v_cruise[i]
            q0i = self.q0[i]

            q = np.zeros(len(t))
            qd = np.zeros(len(t))
            qdd = np.zeros(len(t))

            q_end_ramp = q0i + 0.5 * a * self.t_ramp**2
            q_end_cruise = q_end_ramp + vc * (self.t_cruise_end - self.t_ramp)

            for k in range(len(t)):
                tk = t[k]
                if tk <= self.t_ramp:
                    qdd[k] = a
                    qd[k] = a * tk
                    q[k] = q0i + 0.5 * a * tk**2
                elif tk <= self.t_cruise_end:
                    qdd[k] = 0.0
                    qd[k] = vc
                    q[k] = q_end_ramp + vc * (tk - self.t_ramp)
                else:
                    dt = tk - self.t_cruise_end
                    qdd[k] = -a
                    qd[k] = vc - a * dt
                    q[k] = q_end_cruise + vc * dt - 0.5 * a * dt**2

            X[i, 0, :] = q
            X[i, 1, :] = qd
            X[i, 2, :] = qdd

        return t, X




def main():
    ndof = 2
    method = TrapezoidalTrajectory(ndof=ndof)
    mode = "joint"

    # --------------------------------------------------------
    # Point-to-point multi-axis trajectory generator
    # --------------------------------------------------------

    # traj = MultiAxisTrajectoryGenerator(method=method,
    #                                     mode=mode,
    #                                     ndof=ndof)
    
    # traj.solve(q0=-30, qf=60, T=1)
    # traj.generate(nsteps=20)

    # --------------------------------------------------------
    # Via point multi-axis trajectory generator
    # --------------------------------------------------------

    traj = MultiSegmentTrajectoryGenerator(method=method,
                                           mode=mode,
                                           ndof=ndof,
                                            )
    via_points = [[-30, 30], [0, 45], [30, 15], [50, -30]]

    traj.solve(via_points, T=2)
    traj.generate(nsteps_per_segment=20)
    
    
    # plotter
    traj.plot()

if __name__ == "__main__":
    main()