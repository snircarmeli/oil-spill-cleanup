import numpy as np
import os
import matplotlib.pyplot as plt

def read_points_from_file(file_path, skip_lines):
    with open(file_path, 'r') as file:
        lines = file.readlines()[skip_lines:]
    # print(f"Reading points from {file_path}, skipping first {skip_lines} lines.")
    # print(lines)
    points = [tuple(map(float, line.strip().split())) for line in lines if line.strip()]
    return np.array(points) if points else None

if __name__ == "__main__":
    spills_folder = "spills"
    obstacles_folder = "obstacles"

    skip_spill_lines = 2 
    skip_obstacle_lines = 3

    spill_files = [f for f in os.listdir(spills_folder) if f.endswith('.txt')]
    obstacle_files = [f for f in os.listdir(obstacles_folder) if f.endswith('.txt')]

    plt.figure(figsize=(10, 10))

    # Plot spills as polygons
    for spill_file in spill_files:
        spill_points = read_points_from_file(os.path.join(spills_folder, spill_file), skip_lines=skip_spill_lines)
        if spill_points is not None and spill_points.shape[0] > 0:
            # Close the polygon by repeating the first point at the end
            polygon = np.vstack([spill_points, spill_points[0]])
            plt.plot(polygon[:, 0], polygon[:, 1], label='Spill' if spill_file == spill_files[0] else "", color='blue', linewidth=2)
            plt.fill(polygon[:, 0], polygon[:, 1], color='blue', alpha=0.3)

    # Plot obstacles as polygons
    for obstacle_file in obstacle_files:
        obstacle_points = read_points_from_file(os.path.join(obstacles_folder, obstacle_file), skip_lines=skip_obstacle_lines)
        if obstacle_points is not None and obstacle_points.shape[0] > 0:
            polygon = np.vstack([obstacle_points, obstacle_points[0]])
            plt.plot(polygon[:, 0], polygon[:, 1], label='Obstacle' if obstacle_file == obstacle_files[0] else "", color='red', linewidth=2)
            plt.fill(polygon[:, 0], polygon[:, 1], color='red', alpha=0.3)

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Spill and Obstacle Points')
    plt.legend()
    plt.grid()
    plt.show()