import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import sin, cos, pi
import json

def animate_all_data(duo_boats_data, time_vecs, size, rudder_L=0.4):
    fig, ax = plt.subplots()
    # Find the maximum and minimum values of x and y for all boats, ignoring nan values
    x_vals = np.array([pos[0] for duo in duo_boats_data for pos in [duo[0]['boat1_pos'][:2], duo[0]['boat2_pos'][:2]]])
    y_vals = np.array([pos[1] for duo in duo_boats_data for pos in [duo[0]['boat1_pos'][:2], duo[0]['boat2_pos'][:2]]])
    x_min, x_max = np.nanmin(x_vals), np.nanmax(x_vals)
    y_min, y_max = np.nanmin(y_vals), np.nanmax(y_vals)
    ax.set_xlim(x_min - 5 * size, x_max + 5 * size)
    ax.set_ylim(y_min - 5 * size, y_max + 5 * size)

    # ax.set_xlim(-20, 20)
    # ax.set_ylim(-20, 20)

    
    # ax.set_xlim(-7.5, 7.5)  
    # ax.set_ylim(-2.5, 7.5)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Duo Boats with Dynamic Rudder Animation")
    ax.set_aspect('equal')

    boat_lines = []  # To store boat geometry lines for all boats
    link_lines = []  # To store links for all duos
    link_dots = []   # To store dots at the edges of links

    for i in range(len(duo_boats_data)):
        line_boat1, = ax.plot([], [], 'b-', label='Boat 1')  # Boat 1 geometry
        line_boat2, = ax.plot([], [], 'r-', label='Boat 2')  # Boat 2 geometry
        boat_lines.append((line_boat1, line_boat2))

        num_links = int(duo_boats_data[0][0]['num_links'])  # Convert to integer
        links = []
        for _ in range(num_links):
            link, = ax.plot([], [], 'g-')
            links.append(link)
        link_lines.append(links)

        dots, = ax.plot([], [], 'ko', markersize=1.5)  # Small black dots
        link_dots.append(dots)

    ax.legend(loc="upper right")

    def transform_vertices(position, angle, eta, rudder_l=0.4):
        angle = angle - pi / 2  # Rotate 90 degrees to align with the boat
        rotation_matrix = np.array([
            [sin(angle ), cos(angle)],
            [cos(angle), -sin(angle)]
        ])

        base_boat_vertices = np.array([
            [-0.5, 1],  # Bow port side
            [-0.5, 0],  # Stern port side
            [0, 0],  # Stern middle
            [rudder_L * sin(eta), -rudder_L * cos(eta)], # End of rudder
            [0, 0],  # Stern middle
            [0.5, 0],  # Stern starboard side
            [0.5, 1],  # Bow starboard side
            [0, 1.5],  # Bow, head of ship
            [-0.5, 1],  # Complete the loop
        ]) * size

        transformed = base_boat_vertices @ rotation_matrix.T  # Rotate
        transformed += position  # Translate
        return transformed

    def init():
        for line_boat1, line_boat2 in boat_lines:
            line_boat1.set_data([], [])
            line_boat2.set_data([], [])
        for links in link_lines:
            for link in links:
                link.set_data([], [])
        for dots in link_dots:
            dots.set_data([], [])
        return [line for pair in boat_lines for line in pair] + [link for links in link_lines for link in links] + link_dots

    L = json.load(open('params.json'))['boom']['link_length']
    def update(frame):
        for i, (line_boat1, line_boat2) in enumerate(boat_lines):
            if frame < len(duo_boats_data[i]):
                boat1_pos = duo_boats_data[i][frame]['boat1_pos'][:2]
                boat1_angle = duo_boats_data[i][frame]['boat1_pos'][2]
                boat1_eta = duo_boats_data[i][frame]['boat1_control'][1]  # Steering angle

                boat2_pos = duo_boats_data[i][frame]['boat2_pos'][:2]
                boat2_angle = duo_boats_data[i][frame]['boat2_pos'][2]
                boat2_eta = duo_boats_data[i][frame]['boat2_control'][1]  # Steering angle

                time = duo_boats_data[i][frame]['time']

                transformed_boat1 = transform_vertices(boat1_pos, boat1_angle, boat1_eta, rudder_L)
                transformed_boat2 = transform_vertices(boat2_pos, boat2_angle, boat2_eta, rudder_L)

                line_boat1.set_data(transformed_boat1[:, 0], transformed_boat1[:, 1])
                line_boat2.set_data(transformed_boat2[:, 0], transformed_boat2[:, 1])

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

                if links_positions:
                    # Convert to NumPy arrays for efficient handling
                    links_positions = np.array(links_positions)
                    dots_positions = np.array(dots_positions)

                    # Update plot data
                    for j, link in enumerate(link_lines[i]):
                        link.set_data(links_positions[2*j:2*j+2, 0], links_positions[2*j:2*j+2, 1])
                    link_dots[i].set_data(dots_positions[:, 0], dots_positions[:, 1])
                else:
                    for link in link_lines[i]:
                        link.set_data([], [])
                    link_dots[i].set_data([], [])

        return [line for pair in boat_lines for line in pair] + [link for links in link_lines for link in links] + link_dots
    # Set interval for real-time simulation based on integration method
    simulation_params = json.load(open('params.json'))['simulation']
    dt = simulation_params["time_step"]
    if simulation_params["integration_method"] == "RK45":
        # Calculate average time interval between each time step for all duos
        interval = 1000 * np.mean([np.mean(np.diff(time_vecs[i])) for i in range(len(duo_boats_data))])  # Interval in milliseconds
    else:
        interval = 1000 * (time_vecs[0][1] - time_vecs[0][0]) # Interval in milliseconds
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    
    # interval = 1e3
    def update_with_time(frame):
        artists = update(frame)
        time_text.set_text(f'Time: {time_vecs[0][frame]:.2f}s')
        return artists + [time_text]

    ani = FuncAnimation(fig, update_with_time, frames=len(time_vecs[0]), init_func=init, blit=True, interval=interval)
    plt.show()