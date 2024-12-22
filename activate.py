import numpy as np
import matplotlib.pyplot as plt
import subprocess
from math import pi, sin, cos
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.animation import FuncAnimation

command = "make"
subprocess.run(command, shell=True, text=True, capture_output=True)
command = "/mnt/c/Users/snir2/OneDrive\ -\ Technion/Msc.\ Electrical\ Engineering/Thesis/code/main.exe"
process = subprocess.run(command, shell=True, text=True, capture_output=True)

out_code = process.returncode
stdout = process.stdout
print("stdout: ", stdout)
print("out_code: ", out_code)

if out_code == 0:
    
   # Path to the file
    file_path = 'output.txt'

    # Initialize an empty list to hold the data
    boat_data = []

    # Open the file and read the contents
    with open(file_path, 'r') as file:
        for line in file:
            # Convert each line to a list of floats
            numbers = [float(num) for num in line.split()]
            # Append the list of floats to the data list
            boat_data.append(numbers)

    # Convert the list of lists into a NumPy array
    boat_data = np.array(boat_data)
    # print(pos)
    x = boat_data[:, 0]
    y = boat_data[:, 1]
    theta = boat_data[:, 2]
    x_dot = boat_data[:, 3]
    y_dot = boat_data[:, 4]
    omega = boat_data[:, 5]
    t = boat_data[:, 6]
    F = boat_data[:, 7]
    eta = boat_data[:, 8]

    dt = t[1] - t[0]
    n = len(x)

    step = 1
    x_anim = x[::step]
    y_anim = y[::step]
    theta_anim = theta[::step]
    t_anim = t[::step]
    eta_anim = eta[::step]
    size = 1e0 # Scaling parameter for size of ship on plot
    # Create the figure and axis
    fig, ax = plt.subplots()
    print(str(min(x_anim)) + " " + str(min(y_anim)))
    print(str(max(x_anim)) + " " + str(max(y_anim)))

    ax.set_xlim([min(x_anim) - size, max(x_anim) + size])
    ax.set_ylim([min(y_anim) - size, max(y_anim) + size])
    # Set equal aspect ratio
    ax.set_aspect('equal')
    line, = ax.plot([], [], 'k-')  # Line object to update the boat's position

    # Initialize the animation by creating an empty plot.
    def init():
        line.set_data([], [])
        return line,

    # Animation function which updates the figure
    def animate(i):
        # Create little boats
        # Create a custom boat marker
        rudder_L = 0.3
        
        boat_vertices = np.array([
            [-0.5, 0.5], # Bow port side
            [-0.5, -0.5], # Stern port side
            [0, -0.5], # Stern middle
            
            [rudder_L * sin(eta_anim[i]), -0.5 - rudder_L * cos(eta_anim[i])], # End of rudder
            [0, -0.5], # Stern middle
            [0.5, -0.5], # Stern stbd side
            [0.5, 0.5], # Bow stbd side
            [0, 1], #Bow, head of ship
            [-0.5, 0.5], # Bow port side, complete vertice circle
        ]) * size
        print(f"Time: {dt * i:.4f} [s]")

        # boat_codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]

        # boat_marker = Path(boat_vertices, boat_codes)
        # Calculate the rotation matrix based on the heading angle theta
        # print(theta_anim[i])
        rotation_matrix = np.array([
            [np.cos(theta_anim[i]), -np.sin(theta_anim[i])],
            [np.sin(theta_anim[i]), np.cos(theta_anim[i])]
        ])
        rotated_vertices = boat_vertices @ rotation_matrix + np.array([x_anim[i], y_anim[i]]).T
        
        line.set_data(rotated_vertices[:, 0], rotated_vertices[:, 1])
        return line,

    # Create the animation
    interval = 1000 * t_anim[-1] / len(t_anim)
    ani = FuncAnimation(fig, animate, frames=len(t_anim), init_func=init, blit=True, interval=interval)

    # To save the animation, you can use:
    # ani.save('boat_animation.mp4', writer='ffmpeg')

    plt.show()