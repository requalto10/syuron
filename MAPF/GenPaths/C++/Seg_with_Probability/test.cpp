#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <tuple>
#include <algorithm>
#include <random>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <functional>

using namespace std;

// 定数定義
const int n = 30;  // グリッドの幅
const int m = 30;  // グリッドの高さ
const int a = 50;  // エージェント数
const int time_limit = 200;  // 最大タイムステップ
const int num_solve = 100;  // 経路探索回数
const int d = 3;  // 距離の閾値
const double constant = 1.0;  // スコア計算用定数
vector<double> k_values;

// カスタムハッシュ関数
struct PairHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto h1 = hash<T1>{}(p.first);
        auto h2 = hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// ハッシュ関数の定義
namespace std {
    template <>
    struct hash<pair<int, int>> {
        size_t operator()(const pair<int, int>& p) const {
            return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };
}

// マンハッタン距離
int heuristic(pair<int, int> a, pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

// ランダム性を加えたA*アルゴリズム
vector<pair<int, int>> randomized_astar(pair<int, int> start, pair<int, int> goal,
                                        vector<vector<int>>& grid,
                                        unordered_map<int, unordered_set<pair<int, int>, PairHash>>& occupied) {
    using Node = tuple<double, int, pair<int, int>, vector<pair<int, int>>>;
    priority_queue<Node, vector<Node>, greater<Node>> open_set;

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis_noise(0.0, 10.0);

    double noise = dis_noise(gen);
    open_set.push(make_tuple(heuristic(start, goal) + noise, 0, start, vector<pair<int, int>>{start}));
    unordered_set<pair<pair<int, int>, int>, PairHash> closed_set;

    while (!open_set.empty()) {
        auto [f_cost, g_cost, current, path] = open_set.top();
        open_set.pop();

        if (current == goal) {
            return path;
        }

        if (closed_set.find(make_pair(current, g_cost)) != closed_set.end()) {
            continue;
        }
        closed_set.insert(make_pair(current, g_cost));

        vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        shuffle(directions.begin(), directions.end(), gen);

        for (auto& dir : directions) {
            int nx = current.first + dir.first;
            int ny = current.second + dir.second;

            if (0 <= nx && nx < n && 0 <= ny && ny < m) {
                pair<int, int> neighbor = make_pair(nx, ny);
                if (grid[ny][nx] == 0 || neighbor == goal) {
                    if (closed_set.find(make_pair(neighbor, g_cost + 1)) == closed_set.end()) {
                        vector<pair<int, int>> new_path = path;
                        new_path.push_back(neighbor);
                        double noise = dis_noise(gen);
                        open_set.push(make_tuple(g_cost + 1 + heuristic(neighbor, goal) + noise,
                                                 g_cost + 1, neighbor, new_path));
                    }
                }
            }
        }
    }
    return {};
}

int main() {
    // k値の設定
    for (double k = 0.5; k <= 50.0; k += 0.5) k_values.push_back(k);

    // エージェントとグリッドの初期化
    vector<vector<int>> grid(m, vector<int>(n, 0));
    random_device rd;
    mt19937 gen(rd());
    vector<pair<int, int>> all_positions;

    for (int x = 0; x < n; ++x)
        for (int y = 0; y < m; ++y)
            all_positions.push_back(make_pair(x, y));

    shuffle(all_positions.begin(), all_positions.end(), gen);
    vector<pair<int, int>> agent_starts(all_positions.begin(), all_positions.begin() + a);
    vector<pair<int, int>> agent_goals(all_positions.begin() + a, all_positions.begin() + 2 * a);

    unordered_map<int, pair<pair<int, int>, pair<int, int>>> agents;
    for (int i = 0; i < a; ++i) agents[i + 1] = make_pair(agent_starts[i], agent_goals[i]);

    unordered_map<pair<pair<int, int>, pair<int, int>>, double, PairHash> segment_counts;

    for (int run = 0; run < num_solve; ++run) {
        unordered_map<int, unordered_set<pair<int, int>, PairHash>> reserved;
        for (auto& [agent_id, positions] : agents) {
            auto path = randomized_astar(positions.first, positions.second, grid, reserved);
            unordered_set<pair<pair<int, int>, pair<int, int>>, PairHash> unique_segments;

            for (size_t t = 0; t < path.size() - 1; ++t) {
                auto segment = make_pair(path[t], path[t + 1]);
                unique_segments.insert(segment);
            }

            for (auto& segment : unique_segments) segment_counts[segment]++;
        }
    }

    // k匿名化とデータ残留率の計算
    int total_segments = segment_counts.size();
    for (double k_value : k_values) {
        int retained_segments = 0;
        for (auto& [segment, count] : segment_counts) {
            double probability = count / static_cast<double>(a * num_solve);
            if (probability >= k_value) retained_segments++;
        }

        double retention_rate = (retained_segments / static_cast<double>(total_segments)) * 100.0;
        cout << "k = " << k_value << ", Data Retention Rate: " << fixed << setprecision(2) << retention_rate << "%\n";
    }

    return 0;
}
