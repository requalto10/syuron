import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq

# グリッドサイズとエージェント数の設定
n = m = 10
a = 5

# エージェントの始点と終点を設定
agents = {
    1: {'start': (0, 0), 'goal': (9, 9)},
    2: {'start': (0, 9), 'goal': (9, 0)},
    3: {'start': (5, 0), 'goal': (5, 9)},
    4: {'start': (0, 5), 'goal': (9, 5)},
    5: {'start': (2, 2), 'goal': (7, 7)}
}

# エージェントの経路を記録する辞書
agent_paths = {}

# グリッドの初期化
grid = np.zeros((n, m))

# A*アルゴリズムの実装
def astar(start, goal, grid, occupied):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, [start]))
    closed_set = set()

    while open_set:
        f_cost, g_cost, current, path = heapq.heappop(open_set)

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
    print(f'Time Step {time_step}:')
    for agent_id in sorted(agent_paths.keys()):
        path = agent_paths[agent_id]
        if time_step < len(path):
            x, y = path[time_step]
            agent_time_positions[agent_id].append((time_step, x, y))
            print(f'  Agent {agent_id}: Position ({x}, {y})')
        else:
            # 目的地に到達済み
            x, y = path[-1]
            agent_time_positions[agent_id].append((time_step, x, y))
            print(f'  Agent {agent_id}: Position ({x}, {y}) [Arrived]')
    print('-' * 30)

# 経路情報をファイルに保存
with open('agent_paths.txt', 'w') as f:
    f.write('agent_id,x,y,time_step\n')
    for agent_id, positions in agent_time_positions.items():
        for time_step, x, y in positions:
            f.write(f'{agent_id},{x},{y},{time_step}\n')

# 経路の3次元プロット
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for agent_id, positions in agent_time_positions.items():
    times = [t for t, x, y in positions]
    xs = [x for t, x, y in positions]
    ys = [y for t, x, y in positions]
    ax.plot(xs, ys, times, label=f'Agent {agent_id}')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('Agent Paths in 3D Space')
ax.legend()
plt.show()
