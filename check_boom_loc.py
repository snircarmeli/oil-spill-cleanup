import numpy as np
import matplotlib.pyplot as plt
import subprocess
from math import pi, sin, cos
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.animation import FuncAnimation
import os
import glob

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

command = "/mnt/c/Users/snir2/OneDrive\ -\ Technion/Msc.\ Electrical\ Engineering/Thesis/code/main.exe"
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

    # Initialize a list to hold the data for each duo
    duo_boats_data = []
    # Read each file and extract the data
    for txt_file in txt_files:
        with open(txt_file, 'r') as file:
            lines = file.readlines()
            duo_data = {
                'boat1_pos': [],
                'boat1_vel': [],
                'boat2_pos': [],
                'boat2_vel': [],
                'num_links': 0,
                'link_length': 0,
                'links_states': []
            }
            for i, line in enumerate(lines):
                values = list(map(float, line.split()))
                if i == 0:
                    duo_data['boat1_pos'] = values[:3]
                    duo_data['boat1_vel'] = values[3:6]
                elif i == 1:
                    duo_data['boat2_pos'] = values[:3]
                    duo_data['boat2_vel'] = values[3:6]
                elif i == 2:
                    duo_data['num_links'] = int(values[0])
                    duo_data['link_length'] = values[1]
                else:
                    duo_data['links_states'].append(values)
            duo_boats_data.append(duo_data)

else:
    print("Simulation failed.")
