import numpy as np
import matplotlib.pyplot as plt



class MultiAxisTrajectoryGenerator():
    """
    Multi-axis trajectory generator for joint or task space trajectories.

    Supports cubic, quintic polynomial, and trapezoidal velocity profiles.
    """
    
    def __init__(self, method=None,
                 mode="joint",
                 ndof=1,
                 ):
        """
        Initialize the trajectory generator with the given configuration.
        """
        if method is None:
            raise ValueError("Trajectory generation method must be specified.")
        
        self.ndof = ndof
        self.method = method
        
        if mode == "joint":
            self.mode = "Joint Space"
            self.labels = [f'axis{i+1}' for i in range(self.ndof)]
        elif mode == "task":
            self.mode = "Task Space"
            self.labels = ['x', 'y', 'z']  


    def solve(self, q0, qf, qd0=None, qdf=None, qdd0=None, qddf=None, T=1):
        """
        Fit the trajectory and solve for the trajectory parameters.

        Args:
            q0   : start position
            qf   : final position
            qd0  : start velocity (optional)
            qdf  : final velocity (optional)
            qdd0 : start acceleration (optional)
            qddf : final acceleration (optional)
            T    : duration of the trajectory
        """
        self.T = T
        self.method.solve(q0, qf, qd0, qdf, T=T)

    
    def generate(self, nsteps=100):
        """
        Generate the trajectory at discrete time steps.

        Args:
            nsteps (int): Number of time steps.
        Returns:
            list: List of position, velocity, acceleration for each DOF.
        """
        self.t, self.X = self.method.generate(tf=self.T, nsteps=nsteps)


    def plot(self):
        """
        Plot the position, velocity, and acceleration trajectories.
        """
        self.fig = plt.figure()
        self.sub1 = self.fig.add_subplot(3,1,1)  # Position plot
        self.sub2 = self.fig.add_subplot(3,1,2)  # Velocity plot
        self.sub3 = self.fig.add_subplot(3,1,3)  # Acceleration plot

        self.fig.set_size_inches(8, 10)    
        self.fig.suptitle(self.mode + " Trajectory Generator (Point-to-Point)", fontsize=16)

        colors = ['r', 'g', 'b', 'm', 'y']

        for i in range(self.ndof):
            # position plot
            self.sub1.plot(self.t, self.X[i][0], colors[i]+'o-', label=self.labels[i])
            self.sub1.set_ylabel('position', fontsize=15)
            self.sub1.grid(True)
            self.sub1.legend()
        
            # velocity plot
            self.sub2.plot(self.t, self.X[i][1], colors[i]+'o-', label=self.labels[i])
            self.sub2.set_ylabel('velocity', fontsize=15)
            self.sub2.grid(True)
            self.sub2.legend()

            # acceleration plot
            self.sub3.plot(self.t, self.X[i][2], colors[i]+'o-', label=self.labels[i])
            self.sub3.set_ylabel('acceleration', fontsize=15)
            self.sub3.set_xlabel('Time (secs)', fontsize=18)
            self.sub3.grid(True)
            self.sub3.legend()

        plt.show()
        

class MultiSegmentTrajectoryGenerator():
    """
    Multi-segment trajectory generator.

    Output:
        t : (N,)
        X : (ndof, 3, N)
    """
    
    def __init__(self, method=None,
                 mode="joint",
                 ndof=1
                 ):
        """
        Initialize the trajectory generator with the given configuration.
        """
        if method is None:
            raise ValueError("Trajectory generation method must be specified.")
        
        self.ndof = ndof
        self.method = method

        if mode == "joint":
            self.mode = "Joint Space"
            self.labels = [f'axis{i+1}' for i in range(self.ndof)]
        elif mode == "task":
            self.mode = "Task Space"
            self.labels = ['x', 'y', 'z']

    
    def solve(self, waypoints, T):
        """
        Fit the trajectories and solve for the trajectory parameters.

        Args:
            waypoints : (N, ndof)
            T         : segment duration
        """

        wp = np.asarray(waypoints, dtype=float)

        self.waypoints = wp
        self.n_segments = wp.shape[0] - 1
        self.T = T

        # Piecewise polynomial case
        self.segment_models = []
        for i in range(self.n_segments):
            # position constraints at waypoints
            q0 = wp[i]
            qf = wp[i + 1]

            # Velocity continuity at waypoints
            if i == 0: # first segment, start velocity is zero
                qd0 = np.zeros(self.ndof)
            else:
                qd0 = (wp[i] - wp[i - 1]) / self.T

            if i == self.n_segments - 1: # last segment, final velocity is zero
                qdf = np.zeros(self.ndof)
            else:
                qdf = (wp[i + 1] - wp[i]) / self.T

            print(f"Segment {i+1}: q0={q0}, qf={qf}, qd0={qd0}, qdf={qdf}")

            model = type(self.method)(ndof=self.ndof) # creates a new instance of the trajectory gen class
            model.solve(q0, qf, qd0, qdf, T=T)
            self.segment_models.append(model)

    
    def generate(self, nsteps_per_segment=100):
        """
        Generate trajectory.

        Args:
            nsteps_per_segment : number of samples per segment

        Returns:
            t : (N,)
            X : (ndof, 3, N)
        """
        segments_X = []

        total_nsteps = (nsteps_per_segment-1) * self.n_segments + 1 # to avoid duplicate points at segment boundaries
        self.t = np.linspace(0, self.T, total_nsteps)

        for i, model in enumerate(self.segment_models):
            _, X_ = model.generate(tf=self.T, nsteps=nsteps_per_segment)

            if i > 0: # Avoid duplicate points at boundaries
                X_ = X_[:, :, 1:]

            segments_X.append(X_)

        # Concatenate all segments
        self.X = np.concatenate(segments_X, axis=2)

    
    def plot(self):
        """
        Plot trajectory (position, velocity, acceleration).

        Args:
            t : (N,) time vector
            X : (ndof, 3, N) trajectory array
        """

        fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        fig.suptitle("Multi-Segment Trajectory", fontsize=16)

        labels = [f'axis{i+1}' for i in range(self.ndof)]
        colors = ['r', 'g', 'b', 'm', 'y']

        for i in range(self.ndof):
            c = colors[i]+'o-'

            axs[0].plot(self.t, self.X[i, 0, :], c, label=labels[i]) # Position
            axs[1].plot(self.t, self.X[i, 1, :], c, label=labels[i]) # Velocity
            axs[2].plot(self.t, self.X[i, 2, :], c, label=labels[i]) # Acceleration

        axs[0].set_ylabel("Position")
        axs[1].set_ylabel("Velocity")
        axs[2].set_ylabel("Acceleration")
        axs[2].set_xlabel("Time (s)")

        for ax in axs:
            ax.grid(True)
            ax.legend()

        plt.tight_layout()
        plt.show()



