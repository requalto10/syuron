import json
import re

def load_literals(literals_path):
    with open(literals_path, 'r') as f:
        literals = json.load(f)
    # 文字列キーを再びタプルキーに変換
    literals = {eval(k): v for k, v in literals.items()}
    return literals

def parse_solution(solution_path):
    with open(solution_path, 'r') as f:
        content = f.readlines()

    # Extract all positive integers (true variables)
    true_vars = []
    for line in content:
        if line.startswith('v '):
            # Extract all integers from the line and filter positive ones
            true_vars.extend([int(x) for x in line.split() if x.isdigit() and int(x) > 0])

    return true_vars

def decode_variable(literals, var_index):
    # Decoding the variable index to a, x, y, t
    for (a, x, y, t), v in literals.items():
        if v == var_index:
            return a, x, y, t
    print(f"Warning: var_index {var_index} not found in literals")
    return None

def reconstruct_paths(true_vars, literals, num_agents, max_time):
    paths = [[] for _ in range(num_agents)]

    for var in true_vars:
        a, x, y, t = decode_variable(literals, var)
        if 0 <= a < num_agents:
            paths[a].append((x, y, t))
        else:
            print(f"Invalid agent index: {a}")

    # Sort paths by time
    for path in paths:
        path.sort(key=lambda p: p[2])

    return paths

def print_paths(paths):
    for i, path in enumerate(paths):
        print(f"Agent {i} path:")
        for x, y, t in path:
            print(f"  Time {t}: ({x}, {y})")
        print()


# 使用例
num_agents = 3
max_time = 15

literals_path = './cnfs/literas/20240702_004259.json'
literals = load_literals(literals_path)
solution_path = './solutions/solution.txt'
true_vars = parse_solution(solution_path)

# for true_var in true_vars:
#     print(decode_variable(literals, true_var))

paths = reconstruct_paths(true_vars, literals, num_agents, max_time)
print_paths(paths)