def format_grids(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    formatted_lines = []
    inside_grid = False
    current_grid = []

    for line in lines:
        stripped_line = line.strip()
        if stripped_line.startswith("Grid:"):
            inside_grid = True
            current_grid = []
            formatted_lines.append("Grid:")
        elif inside_grid:
            if stripped_line.startswith("[") and stripped_line.endswith("]"):
                # This is a row of the grid
                current_grid.append(stripped_line)
            else:
                # End of the grid
                inside_grid = False
                formatted_lines.append("[\n" + "\n".join(current_grid) + "\n]")
                formatted_lines.append(stripped_line)
        else:
            formatted_lines.append(stripped_line)

    # Print the formatted output
    for formatted_line in formatted_lines:
        print(formatted_line)

# Use the function with the path to your file
format_grids('prob-out.txt')