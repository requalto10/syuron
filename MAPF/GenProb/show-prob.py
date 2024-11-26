import numpy as np
import matplotlib.pyplot as plt

# Define a function to create a grid with walls having a transparency of 0.5
def create_grid_with_walls(wall_coords):
    grid = np.zeros((20, 20, 4))  # Create a 4-channel grid (RGBA)
    grid[:, :, 3] = 1  # Set alpha channel to fully opaque
    for (x, y) in wall_coords:
        grid[x, y] = [0, 0, 0, 0.5]  # Set walls to black with 50% transparency
    return grid

# Wall coordinates (example, modify as needed)
wall_coordinates = [
    (16, 0), (16, 1), (16, 2), (16, 3), (16, 4), (16, 5), (16, 6), (16, 7),
    (18, 0), (18, 1), (18, 2), (18, 3), (18, 4), (18, 5), (18, 6), (18, 7),
    (5, 3), (6, 3), (7, 3), (8, 3), (9, 3),
    (5, 6), (6, 6), (7, 6), (8, 6), (9, 6),
    (2, 18), (18, 0), (16, 1), (18, 1), (16, 2), (18, 2), (16, 3), (18, 3), (16, 4), (18, 4), (16, 5), (18, 5), (16, 6), (18, 6), (16, 7), (18, 7),

    # Add more walls as needed
]

# Define the problems
problems = [
    {
        "grid": create_grid_with_walls(wall_coordinates),
        "start_points": [(0, 0), (19, 19), (0, 19), (19, 0), (10, 10), (5, 15), (15, 5), (2, 18), (18, 2), (3, 3)],
        "goal_points": [(19, 19), (0, 0), (19, 19), (0, 0), (15, 15), (10, 5), (5, 10), (12, 12), (7, 7), (14, 14)],
    },
    {
        "grid": create_grid_with_walls(wall_coordinates),
        "start_points": [(0, 0), (19, 19), (0, 19), (19, 0), (10, 10), (5, 15), (15, 5), (2, 18), (18, 2), (3, 3),
                         (4, 4), (15, 19), (0, 10), (10, 0), (7, 7), (8, 8), (2, 5), (5, 2), (17, 17), (1, 1)],
        "goal_points": [(19, 19), (0, 0), (19, 19), (0, 0), (15, 15), (10, 5), (5, 10), (12, 12), (7, 7), (14, 14),
                        (18, 1), (2, 13), (10, 19), (19, 10), (6, 6), (13, 13), (12, 9), (9, 12), (0, 17), (17, 0)],
    },
    {
        "grid": create_grid_with_walls(wall_coordinates),
        "start_points": [(0, 0), (19, 19), (0, 19), (19, 0), (10, 10), (5, 15), (15, 5), (2, 18), (18, 2), (3, 3),
                         (4, 4), (15, 19), (0, 10), (10, 0), (7, 7), (8, 8), (2, 5), (5, 2), (17, 17), (1, 1),
                         (18, 19), (9, 10), (11, 12), (2, 3), (14, 15), (16, 17), (8, 9), (6, 6), (3, 12), (7, 8)],
        "goal_points": [(19, 19), (0, 0), (19, 19), (0, 0), (15, 15), (10, 5), (5, 10), (12, 12), (7, 7), (14, 14),
                        (18, 1), (2, 13), (10, 19), (19, 10), (6, 6), (13, 13), (12, 9), (9, 12), (0, 17), (17, 0),
                        (1, 18), (12, 11), (8, 7), (14, 2), (2, 14), (17, 16), (11, 9), (8, 8), (16, 3), (12, 7)],
    }
]

# Function to plot the grids
def plot_grids(problems):
    num_problems = len(problems)
    fig, axes = plt.subplots(1, num_problems, figsize=(5 * num_problems, 5))

    for ax, problem in zip(axes, problems):
        grid = problem["grid"]
        ax.imshow(grid)
        ax.set_xticks(np.arange(-0.5, grid.shape[1], 1))
        ax.set_yticks(np.arange(-0.5, grid.shape[0], 1))
        ax.grid(which="major", color="black", linestyle="-", linewidth=2)
        ax.set_title("Grid with Walls")
        
        # Mark start and goal points
        for start in problem["start_points"]:
            ax.scatter(start[1], start[0], c='green', s=100, label='Start' if 'Start' not in ax.get_legend_handles_labels()[1] else "")
        for goal in problem["goal_points"]:
            ax.scatter(goal[1], goal[0], c='red', s=100, label='Goal' if 'Goal' not in ax.get_legend_handles_labels()[1] else "")
    
    # Add legend
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')
    
    plt.tight_layout()
    plt.show()

# Plot the grids
plot_grids(problems)
