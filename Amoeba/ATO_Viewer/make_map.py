import json

xlength = 1
ylength = 10

cell_number = 0
output_map_name = 'IPS_map.json'

position_map = {}

for y in range(ylength):
    for x in range(xlength):
        position_map[cell_number] = [y, x]
        cell_number += 1
with open(output_map_name, 'w') as f:
    json.dump(position_map, f)
