import os
import random

# Directory to store the text files.
output_dir = "spills"
os.makedirs(output_dir, exist_ok=True)

# Function to generate random points around a center.
def generate_spill(center_x, center_y, num_points=5):
    points = []
    for _ in range(num_points):
        x = round(center_x + random.uniform(-1.0, 1.0), 2)
        y = round(center_y + random.uniform(-1.0, 1.0), 2)
        points.append((x, y))
    return points

# Generate 20 spills centered on a grid.
grid_size = 5
spills = {}
for i in range(20):
    # # For now, only generate the 17th spill.
    if i != 16:
        continue
    center_x = (i % grid_size) * 10
    center_y = (i // grid_size) * 10
    num_points = random.randint(5, 6)
    spill_volume = round(random.uniform(10.0, 30.0), 1)
    points = generate_spill(center_x, center_y, num_points)
    content = f"{spill_volume}\n{num_points}\n" + "\n".join(f"{x} {y}" for x, y in points)
    spills[f"oilspill_{i+1:03}.txt"] = content

# Write each file.
for filename, content in spills.items():
    file_path = os.path.join(output_dir, filename)
    with open(file_path, "w") as f:
        f.write(content)

print(f"Generated {len(spills)} spills in the '{output_dir}' directory.")
