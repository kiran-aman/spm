# kinematics_animation.py
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import your core kinematics library
import spm_kinematics as spm

def cubic_trajectory(t, t_f, q_f):
    """3rd order polynomial trajectory."""
    s = t / t_f
    return q_f * (3*s**2 - 2*s**3)

def main():
    print("=" * 60)
    print("Coaxial 3-RRR SPM Standalone 3D Visualizer")
    print("=" * 60)

    # 1. Path Profile Setup (Ramping Roll from 0 to 30 degrees)
    t_f    = 5.0
    t_pitch = 0.25 # final time for roll command

    roll_f = np.radians(0.0)
    pitch_f = np.radians(40.0)
    yaw_f = np.radians(900.0)
    N      = int(t_f * 100) # generally
    times  = np.linspace(0, t_f, N)

    roll_traj = np.array([cubic_trajectory(t, t_f, roll_f) for t in times])
    pitch_traj = np.array([cubic_trajectory(np.minimum(t, t_pitch), t_pitch, pitch_f) for t in times])
    yaw_traj   = np.array([cubic_trajectory(t, t_f, yaw_f) for t in times])
    theta_traj  = np.zeros((N, 3))
    rpy_fk_traj = np.zeros((N, 3))

    # 2. Compute Trajectory Positions
    print("Calculating trajectory positions...")
    for k, (roll, pitch, yaw) in enumerate(zip(roll_traj, pitch_traj, yaw_traj)):
        # Tracking Roll on X-axis (Standard Z-Y-X space)
        thetas = spm.ik(roll, pitch, yaw)
        if thetas is None:
            print(f"  [Error] Trajectory forced robot out of bounds at k={k}")
            return
        theta_traj[k]  = thetas
        rpy_fk_traj[k] = np.array([roll, pitch, yaw]) # Perfect tracking path target

    # 3. Initialize Matplotlib 3D Axis Canvas
    print("Initializing 3D render environment...")
    fig = plt.figure(figsize=(9, 9))
    
    # Python version safe method to ensure a clean 3D axis initialization
    ax = fig.add_subplot(111, projection='3d')

    # Establish locked, stable boundaries so the canvas doesn't auto-scale
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    
    ax.set_xlabel('Global X Axis (Roll axis)')
    ax.set_ylabel('Global Y Axis')
    ax.set_zlabel('Global Z Axis (Coaxial Base axis)')
    ax.set_title("Coaxial 3-RRR SPM Trajectory Motion Profile", fontsize=12, fontweight='bold')

    # 4. Generate Visual Link Components
    # Leg coloring: Red, Green, Blue for Legs 1, 2, 3
    leg_colors = ['#d62728', '#2ca02c', '#1f77b4']
    leg_lines  = [ax.plot([], [], [], '-o', color=leg_colors[i], linewidth=4, markersize=8)[0] for i in range(3)]
    
    # Top platform represented as a solid black structural loop
    platform_line, = ax.plot([], [], [], 'k-o', linewidth=3, markersize=6)

    def init():
        """Clears the visual traces on canvas reload."""
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
        
        # Pull vector locations directly from your core module library file
        coords = spm.get_robot_coordinates(thetas, r, p, y)
        
        plat_x, plat_y, plat_z = [], [], []
        
        for i, leg in enumerate(coords):
            # Extract nodes: Base Origin (0,0,0) -> Elbow Point -> Top Joint Hub
            xs = [leg['base'][0], leg['elbow'][0], leg['platform'][0]]
            ys = [leg['base'][1], leg['elbow'][1], leg['platform'][1]]
            zs = [leg['base'][2], leg['elbow'][2], leg['platform'][2]]
            
            leg_lines[i].set_data(xs, ys)
            leg_lines[i].set_3d_properties(zs)
            
            # Record platform hinge coordinates
            plat_x.append(leg['platform'][0])
            plat_y.append(leg['platform'][1])
            plat_z.append(leg['platform'][2])
            
        # Tie the platform loop closed back to the original index point
        plat_x.append(plat_x[0])
        plat_y.append(plat_y[0])
        plat_z.append(plat_z[0])
        
        platform_line.set_data(plat_x, plat_y)
        platform_line.set_3d_properties(plat_z)
        
        return leg_lines + [platform_line]

    # 5. Build and Save the Playback Loop
    print("Compiling animation frames...")
    ani = animation.FuncAnimation(
        fig, update, frames=N, init_func=init, blit=True, interval=25, repeat=True
    )

    # Clean multi-platform save path handler
    save_dir = os.path.expanduser("~/Documents/spm/spm_ws/validation")
    os.makedirs(save_dir, exist_ok=True)
    gif_path = os.path.join(save_dir, "robot_motion_profile.gif")
    
    # Render and store the asset locally
    ani.save(gif_path, writer='pillow', fps=40)
    print(f"Success! 3D trajectory playback file saved to:\n  {gif_path}")
    
plt.show()

if __name__ == "__main__":
    main()
