from math import sqrt
from typing import List

route = [
    [401.0802917480469, -719.5579833984375],
    [401.6750183105469, -720.3551635742188],
    [403.8046875, -720.33837890625],
    [403.8046875, -720.33837890625],
    [405.9045715332031, -719.7330932617188],
    [408.3725891113281, -719.6593017578125],
    [410.5361328125, -719.9690551757812],
    [412.6141662597656, -720.451904296875],
    [414.6300048828125, -721.0933837890625],
    [417.0857238769531, -721.9415283203125],
    [417.0857238769531, -721.9415283203125],
    [418.8014831542969, -722.4219970703125],
    [420.48101806640625, -722.6392211914062],
    [422.05426025390625, -722.5167236328125],
    [423.4510498046875, -722.0089111328125],
    [425.740478515625, -719.7430419921875],
    [425.740478515625, -719.7430419921875],
    [426.7996826171875, -718.0465698242188],
    [427.85888671875, -716.3500366210938],
    [428.9180908203125, -714.653564453125],
    [429.9772644042969, -712.95703125],
    [431.0364685058594, -711.2605590820312],
    [432.0956726074219, -709.5640869140625],
    [433.1548767089844, -707.8675537109375],
    [434.2140808105469, -706.1710815429688],
    [435.2732849121094, -704.474609375],
    [436.3343505859375, -702.7756958007812],
    [437.44110107421875, -701.06201171875],
    [438.5581970214844, -699.3850708007812],
    [439.6795654296875, -697.7221069335938],
    [440.79998779296875, -696.0658569335938],
    [441.9134826660156, -694.412841796875],
    [443.0084533691406, -692.762451171875],
    [444.0732421875, -691.0994262695312],
    [445.14385986328125, -689.41015625],
    [446.2145080566406, -687.7208251953125],
    [447.2851257324219, -686.031494140625],
    [448.3557434082031, -684.3422241210938],
    [449.4263610839844, -682.6528930664062],
    [450.49700927734375, -680.963623046875],
    [451.567626953125, -679.2742919921875],
    [452.63824462890625, -677.5849609375],
    [453.7088623046875, -675.8956909179688]
]


speed_kmh = 15
fps = 30
time_offset = 0
csv_path = "vehicle1_route.csv"
header = "x_loc [m],y_loc [m],Time [s]"

###########################################

# Helpers

import numpy as np

def resample(data, S):
    """
    Resample a list of [x, y, t] lists to have one list every S seconds.
    
    Parameters:
    - data: List of lists [x, y, t] where x, y are coordinates in meters and t is time in seconds
    - S: Sampling interval in seconds
    
    Returns:
    - List of resampled lists [x, y, t]
    """
    if not data:
        return []
    
    # Sort by time to ensure correct ordering
    data_sorted = sorted(data, key=lambda p: p[2])
    
    # Extract x, y, t arrays
    x_vals = np.array([p[0] for p in data_sorted])
    y_vals = np.array([p[1] for p in data_sorted])
    t_vals = np.array([p[2] for p in data_sorted])
    
    # Create new time points at intervals of S
    t_start = t_vals[0]
    t_end = t_vals[-1]
    t_new = np.arange(t_start, t_end + S, S)
    
    # Interpolate x and y at new time points
    x_new = np.interp(t_new, t_vals, x_vals)
    y_new = np.interp(t_new, t_vals, y_vals)
    
    # Create list of resampled lists
    resampled = [[x_new[i], y_new[i], t_new[i]] for i in range(len(t_new))]
    
    return resampled

def distance(t1: List[float], t2: List[float]) -> float:
    return sqrt((t2[0] - t1[0]) ** 2 + (t2[1] - t1[1]) ** 2)


# Step 1: calculate time at each point
speed_ms = speed_kmh / 3.6
prev_time = time_offset
for p1, p2 in zip(route[:-1], route[1:]):
    dist = distance(p1, p2)
    time = dist / speed_ms + prev_time
    prev_time = time
    p2.append(time)

# the route starts at the time_offset time
route[0].append(time_offset)


# Step 2: interpolate to one datapoint per frame (controlled by `fps` variable)
route = resample(route, 1/fps)

# Step 3: invert y axis to match CARLA coordinate system
for p in route:
    p[1] = -p[1]

with open(csv_path, "w") as csv:
    print(header, file=csv)
    for p in route:
        print(*p, sep=",", file=csv)

print(f"Written route to {csv_path}")
