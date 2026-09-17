import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import spm_kinematics as spm

def cubic_trajectory(t, t_f, q_f):
    """
    starts and ends at zero velocity
    q(t) = q_f * (3*(t/t_f)^2 - 2*(t/t_f)^3)
    """
    s = t / t_f
    return q_f * (3*s**2 - 2*s**3)

def main():
    # motion profile set up
    t_f    = 2.0 # final time general
    t_pitch = 0.25 # final time for roll command

    roll_f = np.radians(0.0) # final positions
    pitch_f = np.radians(30.0)
    yaw_f = np.radians(0.0)
    N      = int(t_f * 100) # generally
    times  = np.linspace(0, t_f, N)

    # trajectory generation
    roll_traj = np.array([cubic_trajectory(t, t_f, roll_f) for t in times])
    pitch_traj = np.array([cubic_trajectory(t, t_f, pitch_f) for t in times])
    # pitch_traj = np.array([cubic_trajectory(np.minimum(t, t_pitch), t_pitch, pitch_f) for t in times])
    yaw_traj   = np.array([cubic_trajectory(t, t_f, yaw_f) for t in times])
    theta_traj  = np.zeros((N, 3))
    rpy_fk_traj = np.zeros((N, 3))

    # compute trajectory positions
    for k, (roll, pitch, yaw) in enumerate(zip(roll_traj, pitch_traj, yaw_traj)):
        thetas = spm.ik(roll, pitch, yaw)
        if thetas is None:
            print(f"  [Error] Trajectory forced robot out of bounds at k={k}")
            return
        theta_traj[k]  = thetas
        rpy_fk_traj[k] = np.array([roll, pitch, yaw])

    # initialize matplotlib 3d axis canvas
    fig = plt.figure(figsize=(9, 9))
    
    # python version safe method to ensure a clean 3d axis initialization
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    
    ax.set_xlabel('global x axis')
    ax.set_ylabel('global y axis')
    ax.set_zlabel('global z axis')
    ax.set_title("spm motion profile", fontsize=12, fontweight='bold')

    # visual link components
    # leg coloring: Red, Green, Blue for Legs 1, 2, 3
    leg_colors = ['#d62728', '#2ca02c', '#1f77b4']
    leg_lines  = [ax.plot([], [], [], '-o', color=leg_colors[i], linewidth=4, markersize=8)[0] for i in range(3)]
    
    # top platform
    platform_line, = ax.plot([], [], [], 'k-o', linewidth=3, markersize=6)

    def init():
        """Clears the visual traces on canvas reload."""
        # plot origin point
        ax.scatter(0, 0, 0, color='#000000', marker='o', s=50, edgecolors='black', label='Intersection Center', zorder=10)

        for line in leg_lines:
            line.set_data([], [])
            line.set_3d_properties([])
        platform_line.set_data([], [])
        platform_line.set_3d_properties([])
        return leg_lines + [platform_line]

    def update(frame):
        """Calculates and redraws the physical coordinates for each frame."""
        thetas  = theta_traj[frame]
        r, p, y = rpy_fk_traj[frame]
        
        # pull vector locations directly from spm_kinematics.py
        coords = spm.get_robot_coordinates(thetas, r, p, y)
        
        plat_x, plat_y, plat_z = [], [], []
        
        for i, leg in enumerate(coords):
            # base origin (0,0,0) -> elbow -> platform
            xs = [leg['base'][0], leg['elbow'][0], leg['platform'][0]]
            ys = [leg['base'][1], leg['elbow'][1], leg['platform'][1]]
            zs = [leg['base'][2], leg['elbow'][2], leg['platform'][2]]
            
            leg_lines[i].set_data(xs, ys)
            leg_lines[i].set_3d_properties(zs)
            
            plat_x.append(leg['platform'][0])
            plat_y.append(leg['platform'][1])
            plat_z.append(leg['platform'][2])
            
        # tie the platform loop closed back to the original index point
        plat_x.append(plat_x[0])
        plat_y.append(plat_y[0])
        plat_z.append(plat_z[0])
        
        platform_line.set_data(plat_x, plat_y)
        platform_line.set_3d_properties(plat_z)
        
        return leg_lines + [platform_line]

    # build/save
    print("compiling animation frames")
    ani = animation.FuncAnimation(
        fig, update, frames=N, init_func=init, blit=True, interval=25, repeat=True
    )

    save_dir = os.path.expanduser("~/Documents/spm/spm_ws/validation")
    os.makedirs(save_dir, exist_ok=True)
    gif_path = os.path.join(save_dir, "robot_motion_profile.gif")
    
    ani.save(gif_path, writer='pillow', fps=40)
    print(f"done:\n  {gif_path}")
    
plt.show()

if __name__ == "__main__":
    main()
