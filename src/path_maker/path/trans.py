"""
Example coordinate conversion utility.

Reads a path file and converts coordinates from EPSG:5179 (Korean TM) to
EPSG:4326 (WGS84 lat/lon). This is a standalone helper script; paths are
hard-coded and should be adapted to your environment before use.
"""

from pyproj import Transformer

# Build a transformer: EPSG:5179 (Korean TM) -> EPSG:4326 (WGS84)
utm5179_to_wgs84 = Transformer.from_crs("EPSG:5179", "EPSG:4326")

# Read the source file (adjust the path to your workspace)
with open("/home/park/ISEV_2023/src/path_maker/path/final_path_first.txt", "r") as f:
    lines = f.readlines()

new_lines = []

# Convert each line and store
for line in lines:
    x, y, z, m = line.strip().split("\t")
    new_x, new_y = utm5179_to_wgs84.transform(float(x), float(y))
    new_line = f"{new_x}\t{new_y}\t{z}\t{m}\n"
    new_lines.append(new_line)

# Write to destination file (adjust the path)
with open("/home/park/ISEV_2023/src/path_maker/path/final_path_first_wgs84.txt", "w") as f:
    f.writelines(new_lines)