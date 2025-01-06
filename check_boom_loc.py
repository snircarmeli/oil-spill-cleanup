import numpy as np
import matplotlib.pyplot as plt
import subprocess
from math import pi, sin, cos
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.animation import FuncAnimation
import os
import glob
import animate_all_data

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
subprocess.run(command, shell=True, text=True, capture_output=True)

command = "make clean"
print("Removing object files ...\n")
subprocess.run(command, shell=True, text=True, capture_output=True)

# Simulation time 
T = 15
# Time step
dt = 0.01
command = f"/mnt/c/Users/snir2/OneDrive\ -\ Technion/Msc.\ Electrical\ Engineering/Thesis/code/main.exe {T} {dt}"
print("Running the main.exe file...\n")
process = subprocess.run(command, shell=True, text=True)
# print(process.stdout)

out_code = process.returncode
# stdout = process.stdout
# print("stdout: ", stdout)

if out_code == 0:
    print("Simulation completed successfully\n")
    # Path to the file
    foldername = 'DuosData'
    folder_path = r'/mnt/c/Users/snir2/OneDrive - Technion/Msc. Electrical Engineering/Thesis/code/' + foldername
    # Check if the folder exists
    folder_exists = os.path.exists(folder_path)
    if folder_exists:
        print(f"Data is in the folder {foldername}\n")
    else:
        print(f"Data folder does not exist")
        exit()
    # Number of files in the folder
    txt_files = glob.glob(os.path.join(folder_path, '*.txt'))
    num_txt_files = len(txt_files)
    print(f"Number of files in the folder {foldername}: ", num_txt_files, "\n")

    time_vec = np.arange(0, T, dt)
    # Initialize a list to hold the data for each duo
    duo_boats_data = []
    # Read each file and extract the data
    for i, txt_file in enumerate(txt_files):
        with open(txt_file, 'r') as file:
            lines = file.readlines()

            # Initialize a list for the `i-th` duo
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
                    'links_states': []
                }
                duo_boats_data[i].append([])
                values = list(map(float, line.split()))
                # print(values)
                duo_data['boat1_pos'] = values[:3]
                duo_data['boat1_vel'] = values[3:6]
                duo_data['boat1_control'] = values[6:8]
                duo_data['boat2_pos'] = values[8:11]
                duo_data['boat2_vel'] = values[11:14]
                duo_data['boat2_control'] = values[14:16]

                duo_data['num_links'] = values[16]
                duo_data['link_length'] = values[17]

                for k in range(num_links): # Just location without velocity
                    duo_data['links_states'].append(values[18 + k * 6: 21 + k * 6])

                # print(duo_data)
                duo_boats_data[i][j] = duo_data.copy()
                
            # print(duo_boats_data[i][0]['links_states'])
            # print()
                
    
    print("Data extraction completed successfully\n")
    print("Animating the data...\n")

    # Animation function to all the boats and links
    animate_all_data.animate_all_data(duo_boats_data, time_vec)
    

else:
    print("Simulation failed.")
