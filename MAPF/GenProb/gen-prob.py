# 20x20 Grid Setup for Problems 4, 5, and 6
def create_grid_with_walls():
    grid = [[0 for _ in range(20)] for _ in range(20)]

    # Create 3 straight paths (2 width, 5 length, and 8 length)
    # Path 1: Straight path, 5 length (2 width)
    grid[2][3:8] = [1, 0, 0, 0, 0, 1]
    grid[3][3:8] = [1, 1, 0, 0, 0, 1]  # Enclosed by walls

    # Path 2: Curved path, 5 length (2 width)
    grid[5][1:6] = [1, 0, 0, 1, 1]
    grid[6][1:6] = [1, 1, 0, 0, 1]

    # Path 3: Straight path, 8 length (2 width)
    for i in range(10, 18):
        grid[i][3] = grid[i][5] = 1
    for i in range(10, 18):
        grid[i][4] = 0  # Path enclosed by walls

    # Path 4: Curved path, 8 length (2 width)
    grid[10][3:7] = [1, 0, 0, 1]  # Adjusted from previous error
    grid[11][3:7] = [1, 1, 0, 0]  # Adjusted to prevent out-of-bounds

    # Additional 5 walls placed randomly (starting and goal points excluded)
    grid[7][7] = 1
    grid[8][6] = 1
    grid[9][5] = 1
    grid[11][8] = 1
    grid[12][9] = 1

    return grid

# Agent Start and Goal Points Setup
start_points = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7), (8, 8), (9, 9)]
goal_points = [(19, 19), (18, 18), (17, 17), (16, 16), (15, 15), (14, 14), (13, 13), (12, 12), (11, 11), (10, 10)]

# Problem Configurations
problems = []

# Problem 4: 20 agents, no walls
problem_4 = {
    'grid': create_grid_with_walls(),
    'agents': 20,
    'start_points': start_points[:20],
    'goal_points': goal_points[:20]
}

# Problem 5: 30 agents, no walls
problem_5 = {
    'grid': create_grid_with_walls(),
    'agents': 30,
    'start_points': start_points[:30],
    'goal_points': goal_points[:30]
}

# Problem 6: 30 agents, with complex wall layout
problem_6 = {
    'grid': create_grid_with_walls(),
    'agents': 30,
    'start_points': start_points[:30],
    'goal_points': goal_points[:30]
}

problems.append(problem_4)
problems.append(problem_5)
problems.append(problem_6)

# Output the complete problem set for copying
for i, problem in enumerate(problems, start=4):
    print(f"Problem {i}:")
    print("Grid:")
    for row in problem['grid']:
        print(''.join(str(cell) for cell in row))
    print(f"Agents: {problem['agents']}")
    print(f"Start Points: {problem['start_points']}")
    print(f"Goal Points: {problem['goal_points']}")
    print("\n")
