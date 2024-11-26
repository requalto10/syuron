import json
import sys
from collections import defaultdict

# usage
# python3 SatPie_finder.py <unsat_solution_file> <var_index_file>

# ��リッドサイズと最大時間ステップを定��
GRID_WIDTH = 6

def read_unsat_solution(unsat_solution_file):
    with open(unsat_solution_file, 'r') as f:
        solution = [int(line.strip()) for line in f if line.strip()]
    print(f"読み込んだUNSAT解の変数の数: {len(solution)}")
    return solution

def load_var_index(var_index_file):
    with open(var_index_file, 'r') as f:
        var_index = json.load(f)
    print(f"読み込んだvar_indexのエントリ数: {len(var_index)}")
    return {int(v): eval(k) for k, v in var_index.items()}

def reconstruct_paths(unsat_solution, var_index):
    paths = defaultdict(list)
    
    for var in unsat_solution:
        if var > 0 and var in var_index:
            agent, x, y, t = var_index[var]
            paths[agent].append((x, y, t))
    
    # 各エージェントの経路を時間順にソート
    for agent in paths:
        paths[agent].sort(key=lambda x: x[2])
    
    print(f"再構成されたパスの数: {len(paths)}")
    return dict(paths)

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
        print("使用方法: python reconstruct_paths_from_unsat.py <unsat_solution_file> <var_index_file>")
        sys.exit(1)

    unsat_solution_file = sys.argv[1]
    var_index_file = sys.argv[2]

    try:
        unsat_solution = read_unsat_solution(unsat_solution_file)
        var_index = load_var_index(var_index_file)
    except FileNotFoundError as e:
        print(f"エラー: ファイルが見つかりません - {e}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"エラー: var_indexファイルの形式が無効です")
        sys.exit(1)

    paths = reconstruct_paths(unsat_solution, var_index)
    print_paths(paths)

if __name__ == "__main__":
    main()