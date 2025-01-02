import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import sin, cos, pi

# Function that takes a list of data files and a time vector and animates them. 
# the data file duo_boats_data should be a list with the following structure:  
# duo_boats_data[i] = [[boat1_pos, boat1_vel, boat1_control, 
# boat2_pos, boat2_vel, boat2_control,
# num_links, link_length, links_states], [...], ...]
# where each list contains the data for a single time step. duo_boats_data[i] is
# the data for the i-th duo.
# The time vector should be a list of the same length as duo_boats_data.
# Animate the data for all the duos in duo_boats_data in a single plot.

def animate_all_data(duo_boats_data, time_vec, size=1.0, rudder_L=0.4):
    # Create a figure and axis object
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)  # Adjust based on your data range
    ax.set_ylim(-10, 10)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Duo Boats with Dynamic Rudder Animation")
    ax.set_aspect('equal')
    
    # Create empty plots for boats and links
    boat_lines = []  # To store boat geometry lines for all boats
    link_lines = []  # To store links for all duos
    link_dots = []   # To store dots at the edges of links

    # Initialize lines for all duos and boats
    for _ in range(len(duo_boats_data)):
        # Boat 1 and Boat 2 lines
        line_boat1, = ax.plot([], [], 'b-', label='Boat 1')  # Boat 1 geometry
        line_boat2, = ax.plot([], [], 'r-', label='Boat 2')  # Boat 2 geometry
        boat_lines.append((line_boat1, line_boat2))
        
        # Links
        links, = ax.plot([], [], 'g-', label='Links')
        link_lines.append(links)
        
        # Dots at the edges of links
        dots, = ax.plot([], [], 'ko', markersize=1.5)  # Small black dots
        link_dots.append(dots)

    # Add legend
    ax.legend(loc="upper right")

    def transform_vertices(position, angle, eta, rudder_l = 0.4):
        """Rotate, translate, and adjust rudder vertices."""
        rotation_matrix = np.array([
            [cos(angle), -sin(angle)],
            [sin(angle), cos(angle)]
        ])

        # Boat vertices (static in local coordinates)
        base_boat_vertices = np.array([
            [-0.5, 1],  # Bow port side
            [-0.5, 0],  # Stern port side
            [0, 0],  # Stern middle
            [rudder_L * sin(eta), - rudder_L * cos(eta)], # End of rudder
            [0, 0],  # Stern middle
            [0.5, 0],  # Stern starboard side
            [0.5, 1],  # Bow starboard side
            [0, 1.5],  # Bow, head of ship
            [-0.5, 1],  # Complete the loop
        ]) * size

        transformed = base_boat_vertices @ rotation_matrix.T  # Rotate

        transformed += position  # Translate
        return transformed

    # Initialization function
    def init():
        for line_boat1, line_boat2 in boat_lines:
            line_boat1.set_data([], [])
            line_boat2.set_data([], [])
        for link in link_lines:
            link.set_data([], [])
        for dots in link_dots:
            dots.set_data([], [])
        return [line for pair in boat_lines for line in pair] + link_lines + link_dots

    # Update function for each frame
    def update(frame):
        L = duo_boats_data[0][frame]['link_length']
        for i, (line_boat1, line_boat2) in enumerate(boat_lines):
            if frame < len(duo_boats_data[i]):
                # Get boat positions, orientations, and steering angles
                boat1_pos = duo_boats_data[i][frame]['boat1_pos'][:2]
                boat1_angle = duo_boats_data[i][frame]['boat1_pos'][2]
                boat1_eta = duo_boats_data[i][frame]['boat1_control'][1]  # Steering angle

                boat2_pos = duo_boats_data[i][frame]['boat2_pos'][:2]
                boat2_angle = duo_boats_data[i][frame]['boat2_pos'][2]
                boat2_eta = duo_boats_data[i][frame]['boat2_control'][1]  # Steering angle

                # Transform and set data for boats
                transformed_boat1 = transform_vertices(boat1_pos, boat1_angle, boat1_eta, rudder_L)
                transformed_boat2 = transform_vertices(boat2_pos, boat2_angle, boat2_eta, rudder_L)

                line_boat1.set_data(transformed_boat1[:, 0], transformed_boat1[:, 1])
                line_boat2.set_data(transformed_boat2[:, 0], transformed_boat2[:, 1])

                # Set data for links
                links_positions = []
                dots_positions = []
                for link_state in duo_boats_data[i][frame]['links_states']:
                    orientation = link_state[2]
                    start_pos = [
                        link_state[0] - cos(orientation) * L / 2,
                        link_state[1] - sin(orientation) * L / 2
                    ]
                    end_pos = [
                        link_state[0] + cos(orientation) * L / 2,
                        link_state[1] + sin(orientation) * L / 2
                    ]
                    links_positions.append(start_pos)
                    links_positions.append(end_pos)
                    dots_positions.append(start_pos)
                    dots_positions.append(end_pos)
                if len(links_positions) > 0:
                    links_positions = np.array(links_positions)
                    dots_positions = np.array(dots_positions)
                    if links_positions.ndim == 2 and links_positions.shape[1] == 2:
                        link_lines[i].set_data(links_positions[:, 0], links_positions[:, 1])
                        link_dots[i].set_data(dots_positions[:, 0], dots_positions[:, 1])

        return [line for pair in boat_lines for line in pair] + link_lines + link_dots

    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(time_vec), init_func=init, blit=True, interval=100)

    # Show the plot
    plt.show()