import datetime
import json

def add_3cnf_clauses(clauses, clause, max_literals=3, next_var=0):
    """
    Adds a clause to the clauses list, breaking it into multiple clauses if necessary to ensure
    each clause has at most max_literals literals.
    
    Parameters:
    - clauses: list of existing clauses
    - clause: clause to add (list of literals)
    - max_literals: maximum number of literals in a clause
    - next_var: the next available auxiliary variable index

    Returns:
    - next_var: updated next available auxiliary variable index
    """
    while len(clause) > max_literals:
        new_clause = clause[:max_literals-1] + [next_var]
        clauses.append(new_clause)
        clause = [-next_var] + clause[max_literals-1:]
        next_var += 1
    clauses.append(clause)
    return next_var

def create_probsat_cnf_file(grid, agents, start_points, goal_points, waypoints, output_file='probsat.cnf', max_time=None):
    clauses = []
    width = len(grid[0])
    height = len(grid)
    max_time = max_time if max_time else width * height

    global var_index # var_indexをprintするため
    var_index = {}
    current_index = 1
    next_var = current_index

    def get_var_index(a, x, y, t):
        nonlocal current_index
        key = (a, x, y, t)
        if key not in var_index:
            var_index[key] = current_index
            current_index += 1
        return var_index[key]

    def is_valid_point(x, y):
        return 0 <= x < width and 0 <= y < height and grid[y][x] != '@'

    # 各エージェントは常に1つの位置にのみ存在する (ハード制約1)
    for a in range(len(agents)):
        for t in range(max_time):
            clause = [get_var_index(a, x, y, t) for y in range(height) for x in range(width) if grid[y][x] != '@']
            if clause:
                next_var = add_3cnf_clauses(clauses, clause, next_var=next_var)
            for i in range(len(clause)):
                for j in range(i + 1, len(clause)):
                    next_var = add_3cnf_clauses(clauses, [-clause[i], -clause[j]], next_var=next_var)

    # 各セルには同時に1つのエージェントしか存在できない (ハード制約2)
    for t in range(max_time):
        for y in range(height):
            for x in range(width):
                if grid[y][x] != '@':
                    for a1 in range(len(agents)):
                        for a2 in range(a1 + 1, len(agents)):
                            next_var = add_3cnf_clauses(clauses, [-get_var_index(a1, x, y, t), -get_var_index(a2, x, y, t)], next_var=next_var)

    # エージェントは隣接するセルにのみ移動できる (ハード制約３)
    directions = [(0, 0), (0, 1), (0, -1), (1, 0), (-1, 0)]
    for a in range(len(agents)):
        for t in range(max_time - 1):
            for y in range(height):
                for x in range(width):
                    if grid[y][x] != '@':
                        clause = [-get_var_index(a, x, y, t)]
                        for dx, dy in directions:
                            nx, ny = x + dx, y + dy
                            if is_valid_point(nx, ny):
                                clause.append(get_var_index(a, nx, ny, t + 1))
                        next_var = add_3cnf_clauses(clauses, clause, next_var=next_var)

    # 各エージェントはスタート地点から開始する (ハード制約４)
    for a, (sx, sy) in enumerate(start_points):
        if is_valid_point(sx, sy):
            next_var = add_3cnf_clauses(clauses, [get_var_index(a, sx, sy, 0)], next_var=next_var)
            for x in range(width):
                for y in range(height):
                    if x==sx and y==sy:
                        continue
                    if grid[y][x] != '@':
                        next_var = add_3cnf_clauses(clauses, [-get_var_index(a, x, y, 0)], next_var=next_var)
        else:
            raise ValueError(f"Invalid start point for agent {a}: ({sx}, {sy})")

    # 各エージェントは最終時刻にゴール地点に存在する (ハード制約５)
    for a, (gx, gy) in enumerate(goal_points):
        if is_valid_point(gx, gy):
            next_var = add_3cnf_clauses(clauses, [get_var_index(a, gx, gy, max_time - 1)], next_var=next_var)
            for x in range(width):
                for y in range(height):
                    if x == gx and y == gy:
                        continue
                    if grid[y][x] != '@':
                        next_var = add_3cnf_clauses(clauses, [-get_var_index(a, x, y, max_time - 1)], next_var=next_var)
        else:
            raise ValueError(f"Invalid goal point for agent {a}: ({gx}, {gy})")

    # 各エージェントは指定された地点を指定された時刻に通過する (ハード制約６)
    for a, agent_waypoints in enumerate(waypoints):
        for wx, wy, t in agent_waypoints:
            if is_valid_point(wx, wy):
                if t < max_time:
                    next_var = add_3cnf_clauses(clauses, [get_var_index(a, wx, wy, t)], next_var=next_var)
                    for x in range(width):
                        for y in range(height):
                            if x == wx and y == wy:
                                continue
                            if grid[y][x] != '@':
                                next_var = add_3cnf_clauses(clauses, [-get_var_index(a, x, y, t)], next_var=next_var)
                else:
                    raise ValueError(f"Invalid waypoint time for agent {a}: ({wx}, {wy}, {t}), exceeds max_time {max_time}")
            else:
                raise ValueError(f"Invalid waypoint for agent {a}: ({wx}, {wy}, {t}), out of bounds or on obstacle")

    # 現在の時刻を取得し、フォーマット
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"./cnfs/{current_time}.cnf"

    # CNFファイルの作成
    with open(output_file, 'w') as f:
        f.write("c Path planning problem for ProbSAT\n")
        f.write("c Generated by gen_cnf.py\n")
        f.write(f'p cnf {next_var - 1} {len(clauses)}\n')
        for clause in clauses:
            f.write(' '.join(map(str, clause)) + ' 0\n')

    # var_index を保存
    var_index_path = f"./cnfs/literas/{current_time}.json"
    with open(var_index_path, 'w') as f:
        json.dump({str(k): v for k, v in var_index.items()}, f)

if __name__ == "__main__":
    grid = [
            '...@..',
            '..@...',
            '..@..@',
            '.@@...',
            '......',
            '..@...'
    ]

    agents = [0]
    start_points = [(0, 0)]
    goal_points = [(5, 0)]

    # 各エージェントの通過点 (x, y, t)
    waypoints = [
        [(4, 2, 10)]
    ]

create_probsat_cnf_file(grid, agents, start_points, goal_points, waypoints, max_time=16)