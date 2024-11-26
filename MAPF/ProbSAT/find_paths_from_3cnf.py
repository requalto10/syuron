import json
import sys
from collections import defaultdict

# グリッドサイズと最大時間ステップを定義
GRID_WIDTH = 6
GRID_HEIGHT = 6
MAX_TIME_STEP = 14

def read_sat_solution(sat_solution_file):
    with open(sat_solution_file, 'r') as f:
        lines = f.readlines()
    
    solution = []
    for line in lines:
        if line.startswith('v'):
            solution.extend(map(int, line.strip().split()[1:]))
    
    print(f"読み込んだSAT解の変数の数: {len(solution)}")
    return solution

def load_var_index(var_index_file):
    with open(var_index_file, 'r') as f:
        var_index = json.load(f)
    print(f"読み込んだvar_indexのエントリ数: {len(var_index)}")
    return {int(v): eval(k) for k, v in var_index.items()}

def reconstruct_paths(sat_solution, var_index):
    paths = defaultdict(list)
    
    for var in sat_solution:
        if var > 0 and var in var_index:
            agent, x, y, t = var_index[var]
            paths[agent].append((x, y, t))
    
    # 各エージェントの経路を時間順にソート
    for agent in paths:
        paths[agent].sort(key=lambda x: x[2])
    
    print(f"再構成されたパスの数: {len(paths)}")
    return dict(paths)

def validate_paths(paths):
    if not paths:
        print("警告: 再構成されたパスが空です。")
        return

    for agent, path in paths.items():
        # 経路の連続性をチェック
        # for i in range(len(path) - 1):
        #     x1, y1, t1 = path[i]
        #     x2, y2, t2 = path[i + 1]
        #     if abs(x2 - x1) + abs(y2 - y1) > 1 or t2 - t1 != 1:
        #         print(f"警告: エージェント{agent}の経路に不連続な移動があります。")
        
        # グリッド範囲内かチェック
        for x, y, _ in path:
            if x < 0 or x >= GRID_WIDTH or y < 0 or y >= GRID_HEIGHT:
                print(f"警告: エージェント{agent}がグリッド範囲外に移動しています。")
        
        # 時間ステップが最大値を超えていないかチェック
        if path[-1][2] > MAX_TIME_STEP:
            print(f"警告: エージェント{agent}の経路が最大時間ステップを超えています。")

def print_paths(paths):
    if not paths:
        print("警告: 表示するパスがありません。")
        return

    for agent, path in paths.items():
        print(f"エージェント {agent} の経路:")
        for x, y, t in path:
            print(f"  時間 {t}: ({x}, {y})")
        print()

def main():
    if len(sys.argv) != 3:
        print("使用方法: python find_paths_from_3cnf.py <sat_solution_file> <var_index_file>")
        sys.exit(1)

    sat_solution_file = sys.argv[1]
    var_index_file = sys.argv[2]

    try:
        sat_solution = read_sat_solution(sat_solution_file)
        var_index = load_var_index(var_index_file)
    except FileNotFoundError as e:
        print(f"エラー: ファイルが見つかりません - {e}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"エラー: var_indexファイルの形式が無効です")
        sys.exit(1)

    paths = reconstruct_paths(sat_solution, var_index)
    validate_paths(paths)
    print_paths(paths)

if __name__ == "__main__":
    main()