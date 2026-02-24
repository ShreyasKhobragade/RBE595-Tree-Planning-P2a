import numpy as np
from scipy.interpolate import splprep, splev, BSpline
import matplotlib.pyplot as plt

class TrajectoryGenerator:
    """
    Generate smooth trajectory from waypoints using  
splines
    Complete implementation with velocity and acceleration profiles
    """
    
    def __init__(self, waypoints):
        self.waypoints = np.array(waypoints)
        self.trajectory_duration = None  # seconds
        self.max_velocity = None  # m/s
        self.max_acceleration = None  # m/s^2

    ##############################################################
    #### TODO - Implement spline trajectory generation ###########
    #### TODO - Ensure velocity and acceleration constraints #####
    #### TODO - Add member functions as needed ###################
    ##############################################################
    def generate_bspline_trajectory(self, num_points=None):
        """
        Generate spline trajectory with complete velocity/acceleration profiles
        
        Parameters:
        - num_points: number of points in the smooth trajectory
        
        Returns:
        - trajectory_points: numpy array of shape (num_points, 3)
        - time_points: numpy array of time stamps
        - velocities: numpy array of velocities (num_points, 3)
        - accelerations: numpy array of accelerations (num_points, 3)
        """
        print("Generating spline trajectory...")

        trajectory_points = None
        time_points = None
        velocities = None
        accelerations = None

        ############## IMPLEMENTATION STARTS HERE ##############
        print("Generating spline trajectory...")
        wp = np.asarray(self.waypoints, dtype=float)
        assert wp.ndim == 2 and wp.shape[1] == 3 and len(wp) >= 2, "Need >=2 waypoints [N,3]"

        # ---- 1) pick limits and duration (defaults if None)
        v_max = 2.0 if self.max_velocity is None else float(self.max_velocity)
        a_max = 1.0 if self.max_acceleration is None else float(self.max_acceleration)

        # ---- 2) fit an interpolating B-spline r(u), u in [0,1]
        # degree k: quintic if enough points, else cubic/linear as needed (m>k)
        m = len(wp)
        k = min(5, max(1, m - 1))
        tck, u_knots = splprep([wp[:,0], wp[:,1], wp[:,2]], s=0.0, k=k)  # exact through waypoints

        # ---- 3) build arc-length lookup S(u) on a dense grid (monotone)
        U = np.linspace(0.0, 1.0, 2001)
        r = np.vstack(splev(U, tck, der=0)).T          # (N,3)
        r1 = np.vstack(splev(U, tck, der=1)).T         # r'(u)
        r1n = np.linalg.norm(r1, axis=1) + 1e-12       # ||r'(u)||
        du = U[1] - U[0]
        dS = 0.5*(r1n[:-1] + r1n[1:]) * du
        S = np.concatenate([[0.0], np.cumsum(dS)])     # cumulative arc length
        L = float(S[-1])                                # total path length

        # ---- 4) pick trajectory duration from speed/accel limits
        # minimal time with trapezoidal profile
        t_acc_min = v_max / a_max
        d_acc = 0.5*a_max*t_acc_min**2
        if L >= 2*d_acc:
            T_min = 2*t_acc_min + (L - 2*d_acc)/v_max
            triangular = False
        else:
            t_peak = np.sqrt(L/a_max)
            T_min = 2*t_peak
            triangular = True

        # If user provided a longer duration, stretch uniformly (keeps within limits).
        if self.trajectory_duration is None or float(self.trajectory_duration) < T_min:
            T = T_min
            v_eff, a_eff = v_max, a_max
        else:
            T = float(self.trajectory_duration)
            scale = T / T_min
            v_eff = v_max / scale
            a_eff = a_max / (scale**2)

        # ---- 5) construct s(t), v(t), a(t) (trapezoid or triangle), vectorized
        def time_profile(T, L, v_eff, a_eff, triangular):
            t = np.linspace(0.0, T, int(max(200, np.ceil(T*50))) if num_points is None else int(num_points))
            if triangular:
                t1 = 0.5*T
                # accel phase
                s = np.empty_like(t); v = np.empty_like(t); a = np.empty_like(t)
                m1 = t <= t1
                s[m1] = 0.5*a_eff*t[m1]**2
                v[m1] = a_eff*t[m1]
                a[m1] = a_eff
                # decel phase
                m3 = ~m1
                tau = T - t[m3]
                s[m3] = L - 0.5*a_eff*tau**2
                v[m3] = a_eff*tau
                a[m3] = -a_eff
            else:
                t_a = v_eff / a_eff
                d_a = 0.5*a_eff*t_a**2
                t_c = (L - 2*d_a) / v_eff
                t1 = t_a
                t2 = t_a + t_c
                s = np.empty_like(t); v = np.empty_like(t); a = np.empty_like(t)
                m1 = t <= t1
                m2 = (t > t1) & (t <= t2)
                m3 = t > t2
                # accel
                s[m1] = 0.5*a_eff*t[m1]**2
                v[m1] = a_eff*t[m1]
                a[m1] = a_eff
                # cruise
                s[m2] = d_a + v_eff*(t[m2] - t1)
                v[m2] = v_eff
                a[m2] = 0.0
                # decel
                tau = T - t[m3]
                s[m3] = L - 0.5*a_eff*tau**2
                v[m3] = a_eff*tau
                a[m3] = -a_eff
            return t, s, v, a

        time_points, s_t, v_t, a_t = time_profile(T, L, v_eff, a_eff, triangular)

        # ---- 6) invert S(u): s -> u (monotone interpolation)
        u_t = np.interp(s_t, S, U)

        # ---- 7) evaluate spline and derivatives at u(t)
        trajectory_points = np.vstack(splev(u_t, tck, der=0)).T                     # (N,3)
        r1_t = np.vstack(splev(u_t, tck, der=1)).T                    # r'(u)
        r2_t = np.vstack(splev(u_t, tck, der=2)).T                    # r''(u)
        r1n_t = np.linalg.norm(r1_t, axis=1) + 1e-12

        # chain rule:
        # u' = v / ||r'(u)||,   u'' = v_dot/||r'|| - (v^2 * <r',r''>) / ||r'||^4
        u_dot  = v_t / r1n_t
        r1_dot_r2 = np.einsum('ij,ij->i', r1_t, r2_t)                 # <r', r''>
        u_ddot = (a_t / r1n_t) - (v_t**2) * r1_dot_r2 / (r1n_t**4)

        velocities    = r1_t * u_dot[:, None]
        accelerations = (r2_t * (u_dot**2)[:, None]) + (r1_t * u_ddot[:, None])

        # ---- 8) final safety pass: uniform time scaling if any residual violation
        v_mag = np.linalg.norm(velocities, axis=1)
        a_mag = np.linalg.norm(accelerations, axis=1)
        sf = 1.0
        if np.any(v_mag > v_eff + 1e-9):
            sf = max(sf, float(np.max(v_mag) / v_eff))
        if np.any(a_mag > a_eff + 1e-9):
            sf = max(sf, float(np.sqrt(np.max(a_mag) / a_eff)))
        if sf > 1.0:
            time_points = time_points * sf
            velocities  = velocities / sf
            accelerations = accelerations / (sf**2)
            T *= sf

        # ---- 9) update fields and return
        self.trajectory_duration = T
        self.max_velocity = v_max    # requested cap
        self.max_acceleration = a_max
        self.max_acceleration = a_max
            
        return trajectory_points, time_points, velocities, accelerations
            

 
    def visualize_trajectory(self, trajectory_points=None, velocities=None, 
                           accelerations=None, ax=None):
        """Visualize the trajectory with velocity and acceleration vectors"""
        if ax is None:
            fig = plt.figure(figsize=(15, 5))
            ax1 = fig.add_subplot(131, projection='3d')
            ax2 = fig.add_subplot(132)
            ax3 = fig.add_subplot(133)
            standalone = True
        else:
            ax1 = ax
            standalone = False
        
        if trajectory_points is not None:
            # Plot 3D trajectory
            ax1.plot(trajectory_points[:, 0], trajectory_points[:, 1], 
                    trajectory_points[:, 2], 'b-', linewidth=2, label='Spline Trajectory')
            
            # Plot waypoints
            ax1.plot(self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2], 
                    'ro-', markersize=8, linewidth=2, label='Waypoints')
            
            # Plot velocity vectors (sampled)
            if velocities is not None:
                step = max(1, len(trajectory_points) // 20)  # Show ~20 vectors
                for i in range(0, len(trajectory_points), step):
                    trajectory_points = trajectory_points[i]
                    vel = velocities[i] * 0.5  # Scale for visualization
                    ax1.quiver(trajectory_points[0], trajectory_points[1], trajectory_points[2], 
                             vel[0], vel[1], vel[2], 
                             color='green', alpha=0.7, arrow_length_ratio=0.1)
            
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('3D Trajectory')
            ax1.legend()
        
        if standalone and velocities is not None and accelerations is not None:
            # Plot velocity magnitude over time
            time_points = np.linspace(0, self.trajectory_duration, len(velocities))
            vel_magnitudes = np.linalg.norm(velocities, axis=1)
            ax2.plot(time_points, vel_magnitudes, 'g-', linewidth=2)
            ax2.axhline(y=self.max_velocity, color='r', linestyle='--', 
                       label=f'Max Vel: {self.max_velocity} m/s')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.set_title('Velocity Profile')
            ax2.grid(True)
            ax2.legend()
            
            # Plot acceleration magnitude over time
            acc_magnitudes = np.linalg.norm(accelerations, axis=1)
            ax3.plot(time_points, acc_magnitudes, 'm-', linewidth=2)
            ax3.axhline(y=self.max_acceleration, color='r', linestyle='--', 
                       label=f'Max Acc: {self.max_acceleration} m/s²')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Acceleration (m/s²)')
            ax3.set_title('Acceleration Profile')
            ax3.grid(True)
            ax3.legend()
            
            plt.tight_layout()
            plt.show()
        
        return ax1 if not standalone else None