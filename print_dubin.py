import matplotlib.pyplot as plt
import json
import os
from math import cos, sin

dubin_path = []
dubin_path_R = []
dubin_path_L = []
# Read points from file "DubinData/dubin_path.txt"
foldername = json.load(open('params.json'))['file_management']['dubin_folder']
filename = "dubin_path.txt"
filename_R = "dubin_path_R.txt"
filename_L = "dubin_path_L.txt"

folder_exists = os.path.exists(foldername)
if folder_exists:
    # print(f"Data is in the folder {foldername}\n")
    pass
else:
    print(f"DubinData folder does not exist")
# Check if all files exist
file_exists = os.path.exists(os.path.join(foldername, filename))
file_exists_R = os.path.exists(os.path.join(foldername, filename_R))
file_exists_L = os.path.exists(os.path.join(foldername, filename_L))

if not file_exists:
    print(f"{filename} file does not exist")
    exit()
if not file_exists_R:
    print(f"{filename_R} file does not exist")
    exit()
if not file_exists_L:
    print(f"{filename_L} file does not exist")
    exit()

# Read the points from the middle file
with open(os.path.join(foldername, filename), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path.append([float(x) for x in line.split()])

# Read the points from the right file
with open(os.path.join(foldername, filename_R), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path_R.append([float(x) for x in line.split()])

# Read the points from the left file
with open(os.path.join(foldername, filename_L), 'r') as file:
    lines = file.readlines()
    for line in lines:
        # If line is empty, 
        dubin_path_L.append([float(x) for x in line.split()])

# find minimal distance between points
if len(dubin_path) > 1:
    min_dist = ((dubin_path[0][0] - dubin_path[1][0])**2 + (dubin_path[0][1] - dubin_path[1][1])**2)**0.5
    for i in range(len(dubin_path)-2):
        dist = ((dubin_path[i][0] - dubin_path[i+1][0])**2 + (dubin_path[i][1] - dubin_path[i+1][1])**2)**0.5
        if dist < min_dist:
            min_dist = dist
else:
    min_dist = 0

arrow_length = min_dist / 2


# print the points on figure, with small arrows using the third number (orientation)
fig, ax = plt.subplots()
for i in range(len(dubin_path) - 1):
    # Print middle path
    ax.plot(dubin_path[i][0], dubin_path[i][1], 'ro')
    ax.arrow(dubin_path[i][0], dubin_path[i][1],
     arrow_length * cos(dubin_path[i][2]), arrow_length * sin(dubin_path[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)
    
    # Print right path
    ax.plot(dubin_path_R[i][0], dubin_path_R[i][1], 'bo')
    ax.arrow(dubin_path_R[i][0], dubin_path_R[i][1],
     arrow_length * cos(dubin_path_R[i][2]), arrow_length * sin(dubin_path_R[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)
    
    # Print left path
    ax.plot(dubin_path_L[i][0], dubin_path_L[i][1], 'go')
    ax.arrow(dubin_path_L[i][0], dubin_path_L[i][1],
     arrow_length * cos(dubin_path_L[i][2]), arrow_length * sin(dubin_path_L[i][2]),
      fc='k', ec='k', head_width=arrow_length / 1.5, head_length=arrow_length / 4)
    
plt.show()
