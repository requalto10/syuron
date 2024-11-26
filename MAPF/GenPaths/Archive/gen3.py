import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq
import random

# グリッドサイズとエージェント数の設定
n = m = 100
a = 100

# パラメータ k の設定
k = 2  # 任意のエージェントが k ステップ以上存在した座標をマーク

# エージェントの始点と終点をランダムに設定
# 始点と終点が重ならないように注意
all_positions = [(x, y) for x in range(n) for y in range(m)]
random.shuffle(all_positions)
agent_starts = all_positions[:a]
agent_goals = all_positions[a:2*a]

agents = {}
for i in range(a):
    agents[i + 1] = {'start': agent_starts[i], 'goal': agent_goals[i]}

# エージェントの経路を記録する辞書
agent_paths = {}

# グリッドの初期化
grid = np.zeros((n, m), dtype=int)

# A*アルゴリズムの実装
def astar(start, goal, grid, occupied):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, [start]))
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

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            nx, ny = neighbor

            if 0 <= nx < n and 0 <= ny < m:
                if grid[ny][nx] == 0 or neighbor == goal:
                    if (neighbor, g_cost + 1) not in closed_set and (neighbor, g_cost + 1) not in occupied.get(g_cost + 1, []):
                        new_path = path + [neighbor]
                        heapq.heappush(open_set, (g_cost + 1 + heuristic(neighbor, goal), g_cost + 1, neighbor, new_path))
    return None

def heuristic(a, b):
    # マンハッタン距離
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# 予約済みの位置を記録する辞書
reserved = {}

# 各エージェントの経路を計算
max_path_length = 0
for agent_id, info in agents.items():
    path = astar(info['start'], info['goal'], grid, reserved)
    if path is None:
        print(f"Agent {agent_id}の経路が見つかりませんでした。")
        continue
    agent_paths[agent_id] = path
    # 予約済み位置の更新
    for t, pos in enumerate(path):
        if t not in reserved:
            reserved[t] = []
        reserved[t].append((pos, agent_id))
    max_path_length = max(max_path_length, len(path))

# シミュレーションの開始
agent_time_positions = {agent_id: [] for agent_id in agents.keys()}

for time_step in range(max_path_length):
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
        print(f'Time Step {time_step} / {max_path_length}')

# 各10ステップの時間区間でエージェントが k ステップ以上存在していた座標をマーク
interval = 10
marked_positions = []

for interval_start in range(0, max_path_length, interval):
    interval_end = interval_start + interval
    position_counts = {}
    position_times = {}

    for agent_id, positions in agent_time_positions.items():
        for t, x, y in positions:
            if interval_start <= t < interval_end:
                pos = (x, y)
                if pos not in position_counts:
                    position_counts[pos] = 0
                    position_times[pos] = []
                position_counts[pos] += 1
                position_times[pos].append(t)

    for pos, count in position_counts.items():
        if count >= k:
            # マークされた座標とその時間（平均時間）を保存
            avg_time = sum(position_times[pos]) / len(position_times[pos])
            marked_positions.append((pos[0], pos[1], avg_time))

# 経路情報をファイルに保存
with open('agent_paths_large.txt', 'w') as f:
    f.write('agent_id,x,y,time_step\n')
    for agent_id, positions in agent_time_positions.items():
        for time_step, x, y in positions:
            f.write(f'{agent_id},{x},{y},{time_step}\n')

# 経路の3次元プロット（エージェント数が多いため、描画に時間がかかる可能性があります）
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# プロットするエージェント数を増やす
plot_agents = random.sample(list(agent_time_positions.keys()), min(30, a))  # 20増やして30に設定

# エージェントの経路をプロット
for agent_id in plot_agents:
    positions = agent_time_positions[agent_id]
    times = [t for t, x, y in positions]
    xs = [x for t, x, y in positions]
    ys = [y for t, x, y in positions]
    ax.plot(xs, ys, times, label=f'Agent {agent_id}')

# マークされた座標をプロット
if marked_positions:
    marked_xs = [x for x, y, t in marked_positions]
    marked_ys = [y for x, y, t in marked_positions]
    marked_ts = [t for x, y, t in marked_positions]
    ax.scatter(marked_xs, marked_ys, marked_ts, color='red', marker='o', label='Marked Positions')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('Agent Paths with Marked Positions (3D)')
ax.legend()
plt.show()
