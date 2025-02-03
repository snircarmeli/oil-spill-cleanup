import matplotlib.pyplot as plt
import json
import os
from math import cos, sin, pi
import time
import subprocess
import numpy as np
from matplotlib.animation import FuncAnimation

################################################################################

# Clear the console in a cross-platform way
import platform
def clear_console():
    if platform.system() == "Windows":
        os.system("cls")
    else:
        os.system("clear")

clear_console()

command = "make"
print("Compiling the code...\n")
start_time = time.time()
subprocess.run(command, shell=True, text=True, capture_output=True)
print(f"Compilation time: {time.time() - start_time:.2f} seconds\n")

command = f"/mnt/c/Users/snir2/OneDrive\ -\ Technion/Msc.\ Electrical\ Engineering/Thesis/code/main_dubin_check.exe"
# print("Running the main.exe file...\n")
start_time = time.time()
process = subprocess.run(command, shell=True, text=True)
running_time = time.time() - start_time

out_code = process.returncode
if out_code != 0:
    print(f"Error in the code. Return code: {out_code}")
    exit()

params = json.load(open('params.json'))

################################################################################

# Extract dubin path from the file
foldername_dubin = params['file_management']['dubin_folder']
filename_dubin = "dubin_path.txt"
filename_dubin_R = "dubin_path_R.txt"
filename_dubin_L = "dubin_path_L.txt"

folder_exists = os.path.exists(foldername_dubin)
if not folder_exists:
    print(f"Dubin folder does not exist")
    exit()
    
# Check if all files exist
file_exists = os.path.exists(os.path.join(foldername_dubin, filename_dubin))
file_exists_dubin_R = os.path.exists(os.path.join(foldername_dubin, filename_dubin_R))
file_exists_dubin_L = os.path.exists(os.path.join(foldername_dubin, filename_dubin_L))

if not file_exists:
    print(f"{filename_dubin} file does not exist")
    exit()
if not file_exists_dubin_R:
    print(f"{filename_dubin_R} file does not exist")
    exit()
if not file_exists_dubin_L:
    print(f"{filename_dubin_L} file does not exist")
    exit()

# Extract boats data from the file
foldername_boats = params['file_management']['output_folder']
filename = "Duo0.txt"
folder_exists = os.path.exists(foldername_boats)
if not folder_exists:
    print(f"Boats folder does not exist")
    exit()

# Extract spill data and convex hull data from the files
foldername_spills = params['file_management']['spills_folder']
foldername_spills_convex = params['file_management']['spills_convex_folder']

folder_exists = os.path.exists(foldername_spills)
if not folder_exists:
    print(f"Spills folder does not exist")
    exit()

folder_exists = os.path.exists(foldername_spills_convex)
if not folder_exists:
    print(f"Spills convex folder does not exist")
    exit()

################################################################################

# Load the data from the dubin files
dubin_path = []
dubin_path_R = []
dubin_path_L = []

# Read the points from the middle file
with open(os.path.join(foldername_dubin, filename_dubin), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path.append([float(x) for x in line.split()])

# Read the points from the right file
with open(os.path.join(foldername_dubin, filename_dubin_R), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path_R.append([float(x) for x in line.split()])

# Read the points from the left file
with open(os.path.join(foldername_dubin, filename_dubin_L), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path_L.append([float(x) for x in line.split()])

################################################################################
# Load the data from the boats file
duo_boats_data = []
with open(os.path.join(foldername_boats, filename), 'r') as file:
    lines = file.readlines()
    duo_boats_data.append([])

    # first 6 numbers are boat1.get_pos(), boat1.get_vel(), next 2 
    # numbers are Force F and steering angle eta.
    # next, 6 numbers are boat2.get_pos and boat2.get_vel(). next 2 
    # numbers are force F and steering angle eta. 
    # After that num_links and L, and then a sequence of every link's position parameters

    values = list(map(float, lines[0].split()))
    num_links = int(values[16])
    link_length = values[17]

    for j, line in enumerate(lines):
        # Variables to hold the data for each duo for a single time step
        duo_data = {
            'boat1_pos': [],
            'boat1_vel': [],
            'boat1_control': [],
            'boat2_pos': [],
            'boat2_vel': [],
            'boat2_control': [],
            'num_links': 0,
            'link_length': 0,
            'links_states': [],
            'time': 0
        }
        values = list(map(float, line.split()))
        # print(values)
        duo_data['boat1_pos'] = values[:3]
        duo_data['boat1_vel'] = values[3:6]
        duo_data['boat1_control'] = values[6:8]
        duo_data['boat2_pos'] = values[8:11]
        duo_data['boat2_vel'] = values[11:14]
        duo_data['boat2_control'] = values[14:16]
        duo_data['num_links'] = int(values[16])
        duo_data['link_length'] = values[17]
        for k in range(num_links): # Just location without velocity
                    duo_data['links_states'].append(values[18 + k * 6: 21 + k * 6])
        #  Time step
        duo_data['time'] = values[-1]
        duo_boats_data[j] = duo_data.copy()

        duo_boats_data.append(duo_data)

################################################################################

# Load the data from the spills file
spills_data = []
# Iterate over all spill files
for spill_file in os.listdir(foldername_spills):
    with open(os.path.join(foldername_spills, spill_file), 'r') as file:
        # Variables to hold the data for each spill for a single time step
        spill_data = {
            'mass': 0,
            'num_points': 0,
            'spill': [],
            'spill_lines': []
        }
        # mass of spill
        values = list(map(float, file.readline().split()))
        spill_data['mass'] = values[0]
        # num of points
        values = list(map(float, file.readline().split()))
        spill_data['num_points'] = int(values[0])
        # points
        for k in range(spill_data['num_points']):
            values = list(map(float, file.readline().split()))
            spill_data['spill'].append([values[0], values[1]])
        # Add lines between every two points
        for k in range(spill_data['num_points']):
            spill_data['spill_lines'].append([spill_data['spill'][k], spill_data['spill'][(k + 1) % spill_data['num_points']]])
        spills_data.append(spill_data)

# Load the data from the spills convex file
spills_convex_data = []
# Iterate over all spill convex files
for spill_convex_file in os.listdir(foldername_spills_convex):
    with open(os.path.join(foldername_spills_convex, spill_convex_file), 'r') as file:
        # Variables to hold the data for each spill for a single time step
        spill_convex_data = {
            'mass': 0,
            'num_points': 0,
            'spill_convex': [],
            'spill_convex_lines': []
        }
        # mass of spill
        values = list(map(float, file.readline().split()))
        spill_convex_data['mass'] = values[0]
        # num of points
        values = list(map(float, file.readline().split()))
        spill_convex_data['num_points'] = int(values[0])
        # points
        for k in range(spill_convex_data['num_points']):
            values = list(map(float, file.readline().split()))
            spill_convex_data['spill_convex'].append([values[0], values[1]])
        # Add lines between every two points
        for k in range(spill_convex_data['num_points']):
            spill_convex_data['spill_convex_lines'].append([spill_convex_data['spill_convex'][k], spill_convex_data['spill_convex'][(k + 1) % spill_convex_data['num_points']]])
        spills_convex_data.append(spill_convex_data)

################################################################################

# find minimal distance between points - for the arrows
if len(dubin_path) > 1:
    min_dist = ((dubin_path[0][0] - dubin_path[1][0])**2 + (dubin_path[0][1] - dubin_path[1][1])**2)**0.5
    for i in range(len(dubin_path)-2):
        dist = ((dubin_path[i][0] - dubin_path[i+1][0])**2 + (dubin_path[i][1] - dubin_path[i+1][1])**2)**0.5
        if dist < min_dist:
            min_dist = dist
else:
    min_dist = 0
arrow_length = min_dist / 2



################################################################################

# Plot all the data
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')

########################################

# Plot all 3 dubin paths, and update the plot with the boats data every time step

# print the points on figure, with small arrows using the third number (orientation)
marker_size = 1.5
for i in range(len(dubin_path) - 1):
    # Print middle path
    ax.plot(dubin_path[i][0], dubin_path[i][1], 'go', markersize=marker_size)
    ax.arrow(dubin_path[i][0], dubin_path[i][1],
     arrow_length * cos(dubin_path[i][2]), arrow_length * sin(dubin_path[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)
    
    # Print right path
    ax.plot(dubin_path_R[i][0], dubin_path_R[i][1], 'bo', markersize=marker_size)
    ax.arrow(dubin_path_R[i][0], dubin_path_R[i][1],
     arrow_length * cos(dubin_path_R[i][2]), arrow_length * sin(dubin_path_R[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)
    
    # Print left path
    ax.plot(dubin_path_L[i][0], dubin_path_L[i][1], 'ro', markersize=marker_size)
    ax.arrow(dubin_path_L[i][0], dubin_path_L[i][1],
     arrow_length * cos(dubin_path_L[i][2]), arrow_length * sin(dubin_path_L[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)

########################################

# Plot the boats

boat_lines = []  # To store boat geometry lines for all boats
link_lines = []  # To store links for all duos
link_dots = []   # To store dots at the edges of links

line_boat1, = ax.plot([], [], 'b-', label='Boat 1')  # Boat 1 geometry
line_boat2, = ax.plot([], [], 'r-', label='Boat 2')  # Boat 2 geometry
boat_lines.append((line_boat1, line_boat2))

num_links = int(duo_boats_data[0]['num_links'])  # Convert to integer
links = []
for _ in range(num_links):
    link, = ax.plot([], [], 'g-')
    links.append(link)
link_lines.append(links)

dots, = ax.plot([], [], 'ko', markersize=1.5)  # Small black dots
link_dots.append(dots)  

########################################

# Plot the spills
for spill in spills_data:
    spill_points = np.array(spill['spill'])
    ax.plot(spill_points[:, 0], spill_points[:, 1], 'r.')
    for spill_line in spill['spill_lines']:
        ax.plot([spill_line[0][0], spill_line[1][0]], [spill_line[0][1], spill_line[1][1]], 'r--')



for spill_convex in spills_convex_data:
    spill_points = np.array(spill_convex['spill_convex'])
    ax.plot(spill_points[:, 0], spill_points[:, 1], 'c--')
    for spill_line in spill_convex['spill_convex_lines']:
        ax.plot([spill_line[0][0], spill_line[1][0]], [spill_line[0][1], spill_line[1][1]], 'c--')


########################################

# Set the limits of the plot
ax.set_xlim(-10, 70)
ax.set_ylim(-30, 30)

size = params['generic_boat']['ship_size']
def transform_vertices(position, angle, eta, rudder_L=0.4):
    angle = angle - pi / 2  # Rotate 90 degrees to align with the boat
    rotation_matrix = np.array([
        [sin(angle), cos(angle)],
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


L = params['boom']['link_length']
rudder_L = params['generic_boat']['rudder_L']

################################################################################

def update(frame):
    for i, (line_boat1, line_boat2) in enumerate(boat_lines):
        if frame < len(duo_boats_data):
            boat1_pos = duo_boats_data[frame]['boat1_pos'][:2]
            boat1_angle = duo_boats_data[frame]['boat1_pos'][2]
            boat1_eta = duo_boats_data[frame]['boat1_control'][1]  # Steering angle

            boat2_pos = duo_boats_data[frame]['boat2_pos'][:2]
            boat2_angle = duo_boats_data[frame]['boat2_pos'][2]
            boat2_eta = duo_boats_data[frame]['boat2_control'][1]  # Steering angle

            time = duo_boats_data[frame]['time']

            transformed_boat1 = transform_vertices(boat1_pos, boat1_angle, boat1_eta, rudder_L)
            transformed_boat2 = transform_vertices(boat2_pos, boat2_angle, boat2_eta, rudder_L)

            line_boat1.set_data(transformed_boat1[:, 0], transformed_boat1[:, 1])
            line_boat2.set_data(transformed_boat2[:, 0], transformed_boat2[:, 1])

            links_positions = []
            dots_positions = []
            for link_state in duo_boats_data[frame]['links_states']:
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

            
            point1 = [
                dubin_path_L[frame][0],
                dubin_path_L[frame][1],
            ]
            point2 = [
                dubin_path_R[frame][0],
                dubin_path_R[frame][1],
            ]
            links_positions.append(point1)
            links_positions.append(point2)
            dots_positions.append(point1)
            dots_positions.append(point2)


            if links_positions:
                # Convert to NumPy arrays for efficient handling
                links_positions = np.array(links_positions)
                dots_positions = np.array(dots_positions)

                # Update plot data
                for j, link in enumerate(link_lines[i]):
                    link.set_data(links_positions[2*j:2*j+2, 0], links_positions[2*j:2*j+2, 1])
                # Add dthe line [point1, point2] to the 
                link_lines[i][-1].set_data(links_positions[-2:, 0], links_positions[-2:, 1])
                link_dots[i].set_data(dots_positions[:, 0], dots_positions[:, 1])
            else:
                for link in link_lines[i]:
                    link.set_data([], [])
                link_dots[i].set_data([], [])
                    
    return [line for pair in boat_lines for line in pair] + [link for links in link_lines for link in links] + link_dots

time_vec = []
for i in range(len(duo_boats_data)):
    time_vec.append(duo_boats_data[i]['time'])

time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
def update_with_time(frame):
    artists = update(frame)
    time_text.set_text(f'Time: {time_vec[frame]:.2f}s')
    return artists + [time_text]

interval = np.mean(np.diff(time_vec)) * 1000  # Interval in milliseconds
# Animate the plot
ani = FuncAnimation(fig, update_with_time, frames=len(duo_boats_data), init_func=init, blit=True, interval=interval)


plt.show()
