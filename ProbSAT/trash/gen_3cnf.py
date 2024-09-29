import datetime
import json

def create_3cnf_clauses(clause, start_index):
    """
    長い節を3-CNF形式に変換します。

    Args:
    clause (list): 変換する元の節
    start_index (int): 新しい変数のための開始インデックス

    Returns:
    tuple: (3-CNF節のリスト, 次に使用可能な変数インデックス)
    """
    if len(clause) <= 3:
        return [clause], start_index
    
    new_clauses = []
    current_index = start_index
    while len(clause) > 3:
        new_var = current_index
        new_clauses.append(clause[:2] + [new_var])
        clause = [-new_var] + clause[2:]
        current_index += 1
    new_clauses.append(clause)
    return new_clauses, current_index

def create_probsat_cnf_file(grid, agents, start_points, goal_points, waypoints, output_file='probsat.cnf', max_time=None):
    """
    MAPFの問題に対して3-CNF形式のCNFファイルを作成します。

    Args:
    grid (list of str): 環境を表すグリッド
    agents (list of int): エージェントのリスト
    start_points (list of tuple): 各エージェントの開始地点 (x, y)
    goal_points (list of tuple): 各エージェントのゴール地点 (x, y)
    waypoints (list of list of tuple): 各エージェントの経由点 (x, y, t)
    output_file (str): 出力CNFファイルの名前
    max_time (int): 考慮する最大時間ステップ
    """
    var_index = {}
    current_index = 1
    width = len(grid[0])
    height = len(grid)
    max_time = max_time if max_time else width * height
    all_clauses = []


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
                new_clauses, current_index = create_3cnf_clauses(clause, current_index)
                all_clauses.extend(new_clauses)

    # 各セルには同時に1つのエージェントしか存在できない (ハード制約2)
    for t in range(max_time):
        for y in range(height):
            for x in range(width):
                if grid[y][x] != '@':
                    for a1 in range(len(agents)):
                        for a2 in range(a1 + 1, len(agents)):
                            all_clauses.append([-get_var_index(a1, x, y, t), -get_var_index(a2, x, y, t)])

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
                        new_clauses, current_index = create_3cnf_clauses(clause, current_index)
                        all_clauses.extend(new_clauses)

    # 各エージェントはスタート地点から開始する (ハード制約４)
    for a, (sx, sy) in enumerate(start_points):
        if is_valid_point(sx, sy):
            all_clauses.append([get_var_index(a, sx, sy, 0)])
            for x in range(width):
                for y in range(height):
                    if (x, y) != (sx, sy) and grid[y][x] != '@':
                        all_clauses.append([-get_var_index(a, x, y, 0)])
        else:
            raise ValueError(f"Invalid start point for agent {a}: ({sx}, {sy})")

    # 各エージェントは最終時刻にゴール地点に存在する (ハード制約５)
    for a, (gx, gy) in enumerate(goal_points):
        if is_valid_point(gx, gy):
            all_clauses.append([get_var_index(a, gx, gy, max_time - 1)])
            for x in range(width):
                for y in range(height):
                    if (x, y) != (gx, gy) and grid[y][x] != '@':
                        all_clauses.append([-get_var_index(a, x, y, max_time - 1)])
        else:
            raise ValueError(f"Invalid goal point for agent {a}: ({gx}, {gy})")

    # 各エージェントは指定された地点を指定された時刻に通過する (ハード制約６)
    for a, agent_waypoints in enumerate(waypoints):
        for wx, wy, t in agent_waypoints:
            if is_valid_point(wx, wy):
                if t < max_time:
                    all_clauses.append([get_var_index(a, wx, wy, t)])
                    for x in range(width):
                        for y in range(height):
                            if (x, y) != (wx, wy) and grid[y][x] != '@':
                                all_clauses.append([-get_var_index(a, x, y, t)])
                else:
                    raise ValueError(f"Invalid waypoint time for agent {a}: ({wx}, {wy}, {t}), exceeds max_time {max_time}")
            else:
                raise ValueError(f"Invalid waypoint for agent {a}: ({wx}, {wy}, {t}), out of bounds or on obstacle")

    # 現在の時刻を取得し、フォーマット
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"./cnfs/{current_time}.cnf"

    # CNFファイルの作成
    with open(output_file, 'w') as f:
        f.write("c Path planning problem for ProbSAT (3-CNF)\n")
        f.write("c Generated by gen_cnf.py\n")
        f.write(f'p cnf {current_index - 1} {len(all_clauses)}\n')
        for clause in all_clauses:
            f.write(' '.join(map(str, clause)) + ' 0\n')

    # var_index を保存
    var_index_path = f"./cnfs/literas/{current_time}.json"
    with open(var_index_path, 'w') as f:
        json.dump({str(k): v for k, v in var_index.items()}, f)

    print(f"CNF file created: {output_file}")
    print(f"Variable index saved: {var_index_path}")

# 使用例
if __name__ == "__main__":
    grid = [
        '.@@@@',
        '.@@@@',
        '.@@@@',
        '.@@@@',
        '.....'
    ]

    agents = [0]
    start_points = [(0, 0)]
    goal_points = [(4, 4)]

    # 各エージェントの通過点 (x, y, t)
    waypoints = [
    
    ]

    create_probsat_cnf_file(grid, agents, start_points, goal_points, waypoints, max_time=9)