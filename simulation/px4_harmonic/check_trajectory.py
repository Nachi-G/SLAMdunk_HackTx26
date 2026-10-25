import math
import yaml

with open('/home/harneksab/hacktx/current_trajectory.txt') as f:
    traj = next(yaml.safe_load_all(f))

times = traj['times']
cx = traj['coeff_x']
cy = traj['coeff_y']
cz = traj['coeff_z']

tree_x, tree_y = 3.0, 0.0
tree_radius = 0.3
drone_radius = 0.4

min_distance = float('inf')
min_z = float('inf')
max_z = float('-inf')

for i in range(len(times) - 1):
    duration = times[i + 1] - times[i]

    for j in range(101):
        t = duration * j / 100

        def evaluate(coeff):
            return (coeff['a'] * t**3 +
                    coeff['b'] * t**2 +
                    coeff['c'] * t +
                    coeff['d'])

        x = evaluate(cx[i])
        y = evaluate(cy[i])
        z = evaluate(cz[i])

        distance = math.hypot(x - tree_x, y - tree_y)

        min_distance = min(min_distance, distance)
        min_z = min(min_z, z)
        max_z = max(max_z, z)

clearance = min_distance - tree_radius - drone_radius

print('Minimum distance to tree center:', round(min_distance, 3), 'm')
print('Estimated horizontal clearance:', round(clearance, 3), 'm')
print('Minimum trajectory altitude:', round(min_z, 3), 'm')
print('Maximum trajectory altitude:', round(max_z, 3), 'm')

if clearance > 0 and min_z >= 0.5 and max_z <= 6.0:
    print('PASS: Sampled trajectory clears the test tree and altitude limits')
else:
    print('WARNING: Sampled trajectory violates a clearance or altitude limit')
