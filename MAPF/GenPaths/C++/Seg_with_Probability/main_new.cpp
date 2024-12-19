#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <fstream>
#include <random>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <tuple>

using namespace std;

// グリッドサイズとエージェント数の設定
const int n = 100;
const int m = 100;
const int a = 300;

// パラメータの設定
vector<int> k_values = {20, 30, 40, 50, 60, 70, 80}; // スコア閾値を%で指定
const int time_limit = 200;   // Time = 0 ~ 200 の範囲のみを描画
const int num_solve = 1000;     // 経路探索の実行回数
const int d = 3;              // 距離の閾値

// タイムスタンプを取得
string getTimestamp() {
    auto now = chrono::system_clock::now();
    time_t t = chrono::system_clock::to_time_t(now);
    tm* local_time = localtime(&t);
    stringstream ss;
    ss << setw(2) << setfill('0') << local_time->tm_mon + 1
       << setw(2) << setfill('0') << local_time->tm_mday << "-"
       << setw(2) << setfill('0') << local_time->tm_hour
       << setw(2) << setfill('0') << local_time->tm_min;
    return ss.str();
}

// カスタムハッシュ関数
struct PairHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        size_t h1, h2;
        
        if constexpr (is_same_v<T1, pair<int, int>>) {
            h1 = hash<int>{}(p.first.first) ^ (hash<int>{}(p.first.second) << 1);
        } else if constexpr (is_same_v<T1, tuple<int, int, int>>) {
            h1 = hash<int>{}(get<0>(p.first)) ^ 
                 (hash<int>{}(get<1>(p.first)) << 1) ^
                 (hash<int>{}(get<2>(p.first)) << 2);
        } else {
            h1 = hash<T1>{}(p.first);
        }
        
        if constexpr (is_same_v<T2, pair<int, int>>) {
            h2 = hash<int>{}(p.second.first) ^ (hash<int>{}(p.second.second) << 1);
        } else if constexpr (is_same_v<T2, tuple<int, int, int>>) {
            h2 = hash<int>{}(get<0>(p.second)) ^ 
                 (hash<int>{}(get<1>(p.second)) << 1) ^
                 (hash<int>{}(get<2>(p.second)) << 2);
        } else {
            h2 = hash<T2>{}(p.second);
        }
        
        return h1 ^ (h2 << 1);
    }
};

struct TupleHash {
    size_t operator()(const tuple<int, int, int>& t) const {
        return hash<int>{}(get<0>(t)) ^
               (hash<int>{}(get<1>(t)) << 1) ^
               (hash<int>{}(get<2>(t)) << 2);
    }
};

// ヒューリスティック関数（マンハッタン距離）
int heuristic(pair<int, int> a, pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

// ランダム性を加えた A* アルゴリズムの実装
vector<pair<int, int>> randomized_astar(pair<int, int> start, pair<int, int> goal,
                                        vector<vector<int>>& grid,
                                        unordered_map<int, unordered_set<pair<int, int>, PairHash>>& occupied) {
    using Node = tuple<double, int, pair<int, int>, vector<pair<int, int>>>;
    priority_queue<Node, vector<Node>, greater<Node>> open_set;

    open_set.push(make_tuple(heuristic(start, goal), 0, start, vector<pair<int, int>>{start}));
    unordered_set<pair<pair<int, int>, int>, PairHash> closed_set;

    const int max_time = 500;  // 最大探索時間

    while (!open_set.empty()) {
        auto [f_cost, g_cost, current, path] = open_set.top();
        open_set.pop();

        if (g_cost > max_time) {
            return {};  // タイムアウト
        }

        if (current == goal) {
            return path;
        }

        if (closed_set.find(make_pair(current, g_cost)) != closed_set.end()) {
            continue;
        }

        closed_set.insert(make_pair(current, g_cost));

        vector<pair<int, int>> neighbors;
        vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        for (auto& dir : directions) {
            int nx = current.first + dir.first;
            int ny = current.second + dir.second;

            if (0 <= nx && nx < n && 0 <= ny && ny < m) {
                pair<int, int> neighbor = make_pair(nx, ny);
                if (grid[ny][nx] == 0 || neighbor == goal) {
                    if (closed_set.find(make_pair(neighbor, g_cost + 1)) == closed_set.end() &&
                        occupied[g_cost + 1].find(neighbor) == occupied[g_cost + 1].end()) {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }

        shuffle(neighbors.begin(), neighbors.end(), mt19937{random_device{}()});

        for (auto& neighbor : neighbors) {
            vector<pair<int, int>> new_path = path;
            new_path.push_back(neighbor);
            open_set.push(make_tuple(g_cost + 1 + heuristic(neighbor, goal),
                                     g_cost + 1, neighbor, new_path));
        }
    }

    return {};
}

int main() {
    vector<pair<int, int>> all_positions;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < m; ++y) {
            all_positions.push_back({x, y});
        }
    }
    shuffle(all_positions.begin(), all_positions.end(), mt19937{random_device{}()});
    vector<pair<int, int>> agent_starts(all_positions.begin(), all_positions.begin() + a);
    vector<pair<int, int>> agent_goals(all_positions.begin() + a, all_positions.begin() + 2 * a);

    vector<vector<int>> grid(m, vector<int>(n, 0));

    unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, int, PairHash> segment_counts;

    for (int run = 0; run < num_solve; ++run) {
        unordered_map<int, unordered_set<pair<int, int>, PairHash>> reserved;

        for (int i = 0; i < a; ++i) {
            auto path = randomized_astar(agent_starts[i], agent_goals[i], grid, reserved);

            for (size_t j = 0; j < path.size() - 1; ++j) {
                segment_counts[{{path[j].first, path[j].second, static_cast<int>(j)},
                                 {path[j + 1].first, path[j + 1].second, static_cast<int>(j + 1)}}]++;
            }
        }
    }

    // スコアの正規化とデータ残留率の計算をここに記載...
    // （省略）

    return 0;
}
