import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import heapq
import random
import math

# グリッドサイズとエージェント数の設定
n = m = 100
a = 100

# パラメータの設定
k = 2           # スコアの閾値
time_limit = 100  # Time = 0 ~ 100 の範囲のみを描画
num_solve = 3     # 経路探索の実行回数
d = 2             # 距離の閾値
constant = 1      # スコア計算時の定数

# エージェントの始点と終点をランダムに設定（全ての実行で共通）
# 始点と終点が重ならないように注意
all_positions = [(x, y) for x in range(n) for y in range(m)]
random.shuffle(all_positions)
agent_starts = all_positions[:a]
agent_goals = all_positions[a:2*a]

# 始点と終点のセットを標準出力に出力
print("Agent Start and Goal Positions:")
for i in range(a):
    print(f"Agent {i+1}: Start {agent_starts[i]}, Goal {agent_goals[i]}")

# エージェント情報の辞書
agents = {}
for i in range(a):
    agents[i + 1] = {'start': agent_starts[i], 'goal': agent_goals[i]}

# グリッドの初期化
grid = np.zeros((n, m), dtype=int)

# ランダム性を加えた A* アルゴリズムの実装
def randomized_astar(start, goal, grid, occupied):
    open_set = []
    # ヒューリスティックにランダムなノイズを加える
    noise = random.uniform(0, 10)
    heapq.heappush(open_set, (heuristic(start, goal) + noise, 0, start, [start]))
    closed_set = set()

    max_time = 500  # 最大探索時間（タイムアウト対策）
    while open_set:
        f_cost, g_cost, current, path = heapq.heappop(open_set)

        if g_cost > max_time:
            return None  # タイムアウト

        if current == goal:
            return path

        if (current, g_cost) in closed_set:
            continue

        closed_set.add((current, g_cost))

        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            nx, ny = neighbor

            if 0 <= nx < n and 0 <= ny < m:
                if grid[ny][nx] == 0 or neighbor == goal:
                    if (neighbor, g_cost + 1) not in closed_set and (neighbor, g_cost + 1) not in occupied.get(g_cost + 1, []):
                        neighbors.append(neighbor)

        random.shuffle(neighbors)  # 隣接ノードの順序をランダムに

        for neighbor in neighbors:
            new_path = path + [neighbor]
            # ヒューリスティックにランダムなノイズを加える
            noise = random.uniform(0, 10)
            heapq.heappush(open_set, (g_cost + 1 + heuristic(neighbor, goal) + noise, g_cost + 1, neighbor, new_path))

    return None

def heuristic(a, b):
    # マンハッタン距離
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# 全実行の経路情報を保存するリスト
all_agent_paths_runs = []

# 複数回の経路探索を実行
for run in range(num_solve):
    print(f"\n--- Run {run+1} ---")
    # 予約済みの位置を記録する辞書
    reserved = {}
    # 各エージェントの経路を記録する辞書
    agent_paths = {}
    max_path_length = 0

    # 各エージェントの経路を計算
    for agent_id, info in agents.items():
        path = randomized_astar(info['start'], info['goal'], grid, reserved)
        if path is None:
            print(f"Agent {agent_id} の経路が見つかりませんでした。")
            continue
        agent_paths[agent_id] = path
        # 予約済み位置の更新
        for t, pos in enumerate(path):
            if t not in reserved:
                reserved[t] = []
            reserved[t].append((pos, agent_id))
        max_path_length = max(max_path_length, len(path))

    # エージェントの経路を保存
    all_agent_paths_runs.append(agent_paths)

# ここからは最後の実行結果を使用して可視化と分析を行う

# 最後の実行結果を取得
agent_paths = all_agent_paths_runs[-1]

# シミュレーションの開始
agent_time_positions = {agent_id: [] for agent_id in agents.keys()}

for time_step in range(min(max_path_length, time_limit + 1)):
    # 各エージェントの位置を更新
    for agent_id in sorted(agent_paths.keys()):
        path = agent_paths[agent_id]
        if time_step < len(path):
            x, y = path[time_step]
            agent_time_positions[agent_id].append((time_step, x, y))
        else:
            # 目的地に到達済み
            x, y = path[-1]
            agent_time_positions[agent_id].append((time_step, x, y))

    # 進捗状況を一定の間隔で表示
    if time_step % 50 == 0:
        print(f'Time Step {time_step} / {time_limit}')

# 全てのセグメントを収集
all_segments = {}

for agent_id, positions in agent_time_positions.items():
    for i in range(len(positions) - 1):
        t1, x1, y1 = positions[i]
        t2, x2, y2 = positions[i + 1]
        segment = ((x1, y1, t1), (x2, y2, t2))
        if segment not in all_segments:
            all_segments[segment] = 0
        all_segments[segment] += 1  # セグメントを通過したエージェント数をカウント

# スコアを計算するための全ての可能なセグメントを生成
possible_segments = []

for t in range(time_limit):
    for x in range(n):
        for y in range(m):
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < n and 0 <= ny < m:
                    possible_segments.append(((x, y, t), (nx, ny, t + 1)))

# セグメントごとのスコアを計算
segment_scores = {}

for seg_a in possible_segments:
    score = 0
    (x1_a, y1_a, t1_a), (x2_a, y2_a, t2_a) = seg_a
    for seg_b, count in all_segments.items():
        (x1_b, y1_b, t1_b), (x2_b, y2_b, t2_b) = seg_b
        # 距離を計算（空間と時間のユークリッド距離）
        space_time_distance = math.sqrt(
            (x1_a - x1_b) ** 2 + (y1_a - y1_b) ** 2 + (t1_a - t1_b) ** 2
        )
        if space_time_distance <= d:
            # スコアを加算
            score += constant * count / (space_time_distance + 1e-6)  # 0除算を防ぐため小さい値を加算
    if score >= k:
        segment_scores[seg_a] = score

# 匿名化後に公開できるセグメント数を計算
num_anony_seg = sum([count for seg, count in all_segments.items() if seg in segment_scores])

# 全ての経路に含まれるセグメント数を計算
num_all_seg = sum([len(agent_paths[agent_id]) - 1 for agent_id in agent_paths])

# データ残留率を計算
data_residual_rate = num_anony_seg / num_all_seg if num_all_seg > 0 else 0

print(f"\nTotal number of segments (num_all_seg): {num_all_seg}")
print(f"Number of anonymous segments (num_anony_seg): {num_anony_seg}")
print(f"Data residual rate: {data_residual_rate:.4f}")

# 経路情報をファイルに保存（最後の実行結果）
with open('agent_paths_limited.txt', 'w') as f:
    f.write('agent_id,x,y,time_step\n')
    for agent_id, positions in agent_time_positions.items():
        for time_step, x, y in positions:
            f.write(f'{agent_id},{x},{y},{time_step}\n')

# 経路の3次元プロット（最後の実行結果）
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# エージェントの経路をプロット（全エージェント）
for agent_id in agent_time_positions.keys():
    positions = agent_time_positions[agent_id]
    times = [t for t, x, y in positions]
    xs = [x for t, x, y in positions]
    ys = [y for t, x, y in positions]
    ax.plot(xs, ys, times, linewidth=0.5, alpha=0.5)

# 始点と終点をプロット
for agent_id in agent_time_positions.keys():
    positions = agent_time_positions[agent_id]
    if positions:
        start_time, x_start, y_start = positions[0]
        end_time, x_end, y_end = positions[-1]
        ax.scatter(x_start, y_start, start_time, color='blue', marker='o')
        ax.scatter(x_end, y_end, end_time, color='blue', marker='^')

# スコアが閾値以上のセグメントを太線でプロット
segments = []
for seg, score in segment_scores.items():
    (x1, y1, t1), (x2, y2, t2) = seg
    avg_time = (t1 + t2) / 2
    segments.append([(x1, y1, t1), (x2, y2, t2)])

# 太線のコレクションを作成
if segments:
    lc = Line3DCollection(segments, colors='red', linewidths=5, alpha=1)
    ax.add_collection(lc)

# Time = 0 ~ 100 の範囲に限定
ax.set_xlim(0, n)
ax.set_ylim(0, m)
ax.set_zlim(0, time_limit)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('Agent Paths with High Traffic Segments (Time 0-100)')
plt.show()
