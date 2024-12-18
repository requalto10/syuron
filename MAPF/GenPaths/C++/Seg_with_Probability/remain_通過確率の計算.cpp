#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <tuple>
#include <algorithm>
#include <fstream>
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
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
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

    template <>
    struct hash<tuple<int, int, int>> {
        size_t operator()(const tuple<int, int, int>& t) const {
            return hash<int>()(get<0>(t)) ^ (hash<int>()(get<1>(t)) << 1) ^ (hash<int>()(get<2>(t)) << 2);
        }
    };

    template <>
    struct hash<pair<pair<int, int>, int>> {
        size_t operator()(const pair<pair<int, int>, int>& p) const {
            return hash<pair<int, int>>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };

    template <>
    struct hash<pair<tuple<int, int, int>, tuple<int, int, int>>> {
        size_t operator()(const pair<tuple<int, int, int>, tuple<int, int, int>>& p) const {
            return hash<tuple<int, int, int>>()(p.first) ^ (hash<tuple<int, int, int>>()(p.second) << 1);
        }
    };
}


// ヒューリスティック関数（マンハッタン距離）
int heuristic(pair<int, int> a, pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

// ランダム性を加えたA*アルゴリズム
vector<pair<int, int>> randomized_astar(pair<int, int> start, pair<int, int> goal,
                                        vector<vector<int>>& grid,
                                        unordered_map<int, unordered_set<pair<int, int>, PairHash>>& occupied,
                                        int& collision_avoidance_count) {
    using Node = tuple<double, int, pair<int, int>, vector<pair<int, int>>>;
    priority_queue<Node, vector<Node>, greater<Node>> open_set;

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis_noise(0.0, 10.0);

    double noise = dis_noise(gen);
    open_set.push(make_tuple(heuristic(start, goal) + noise, 0, start, vector<pair<int, int>>{start}));
    unordered_set<pair<pair<int, int>, int>, PairHash> closed_set;

    const int max_time = 500;  // 最大探索時間

    while (!open_set.empty()) {
        auto [f_cost, g_cost, current, path] = open_set.top();
        open_set.pop();

        if (g_cost > max_time) {
            return {};  // タイムアウト
        }

        if (current == goal) {
            cout << "衝突を回避した回数: " << collision_avoidance_count << endl;
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
                    if (closed_set.find(make_pair(neighbor, g_cost + 1)) == closed_set.end()) {
                        if (occupied[g_cost + 1].find(neighbor) == occupied[g_cost + 1].end()) {
                            neighbors.push_back(neighbor);
                        } else {
                            collision_avoidance_count++;  // 衝突回避をカウント
                        }
                    }
                }
            }
        }

        // 隣接ノードの順序をランダムに
        shuffle(neighbors.begin(), neighbors.end(), gen);

        for (auto& neighbor : neighbors) {
            vector<pair<int, int>> new_path = path;
            new_path.push_back(neighbor);
            double noise = dis_noise(gen);
            open_set.push(make_tuple(g_cost + 1 + heuristic(neighbor, goal) + noise,
                                     g_cost + 1, neighbor, new_path));
        }
    }

    cout << "衝突を回避した回数: " << collision_avoidance_count << endl;
    return {};
}


vector<pair<int, int>> randomized_astar_new(pair<int, int> start, pair<int, int> goal,
                                            vector<vector<int>>& grid) {
    using Node = tuple<double, int, pair<int, int>, vector<pair<int, int>>>;
    priority_queue<Node, vector<Node>, greater<Node>> open_set;

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis_noise(0.0, 10.0);

    double noise = dis_noise(gen);
    open_set.push(make_tuple(heuristic(start, goal) + noise, 0, start, vector<pair<int, int>>{start}));
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
                    if (closed_set.find(make_pair(neighbor, g_cost + 1)) == closed_set.end()) {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }

        // 隣接ノードの順序をランダムに
        shuffle(neighbors.begin(), neighbors.end(), gen);

        for (auto& neighbor : neighbors) {
            vector<pair<int, int>> new_path = path;
            new_path.push_back(neighbor);
            double noise = dis_noise(gen);
            open_set.push(make_tuple(g_cost + 1 + heuristic(neighbor, goal) + noise,
                                     g_cost + 1, neighbor, new_path));
        }
    }

    return {};
}



int main() {
    auto total_start_time = chrono::steady_clock::now();

    for (double k = 1.0; k <= 50.0; k += 0.5) {
        k_values.push_back(k);
    }

    vector<string> output_lines;

    // グリッドとエージェントの初期化
    vector<vector<int>> grid(m, vector<int>(n, 0));
    vector<pair<int, int>> all_positions;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < m; ++y) {
            all_positions.push_back(make_pair(x, y));
        }
    }

    random_device rd;
    mt19937 gen(rd());
    shuffle(all_positions.begin(), all_positions.end(), gen);

    vector<pair<int, int>> agent_starts(all_positions.begin(), all_positions.begin() + a);
    vector<pair<int, int>> agent_goals(all_positions.begin() + a, all_positions.begin() + 2 * a);

    // 始点と終点のセットを標準出力に出力
    stringstream param_info;
    param_info << "# グリッドサイズとエージェント数の設定\n"
               << "n = m = " << n << "\n"
               << "a = " << a << "\n\n"
               << "# パラメータの設定\n"
               << "k_values = {";
    for (size_t i = 0; i < k_values.size(); ++i) {
        param_info << k_values[i];
        if (i != k_values.size() - 1) param_info << ", ";
    }
    param_info << "}  # スコアの閾値のリスト\n"
               << "time_limit = " << time_limit << "  # Time = 0 ~ " << time_limit << " の範囲のみを描画\n"
               << "num_solve = " << num_solve << "     # 経路探索の実行回数\n"
               << "d = " << d << "             # 距離の閾値\n"
               << "constant = " << constant << "      # スコア計算時の定数\n\n";

    cout << param_info.str();
    output_lines.push_back(param_info.str());

    unordered_map<int, pair<pair<int, int>, pair<int, int>>> agents;
    for (int i = 0; i < a; ++i) {
        agents[i + 1] = make_pair(agent_starts[i], agent_goals[i]);
    }

    vector<string> detail_output_lines;

    // 経路探索実行
    vector<unordered_map<int, vector<pair<int, int>>>> all_agent_paths_runs;
    vector<unordered_map<int, vector<pair<int, int>>>> all_agent_paths_runs_new;

    int total_collision_avoidance_count = 0;  // 全エージェントの衝突回避回数の合計

    for (int run = 0; run < num_solve; ++run) {
        if (run % 100 == 0) {
            stringstream run_info;
            run_info << "\n--- Run " << run + 1 << " ---";
            cout << run_info.str() << endl;
            detail_output_lines.push_back(run_info.str());
        }

        unordered_map<int, unordered_set<pair<int, int>, PairHash>> reserved;
        unordered_map<int, vector<pair<int, int>>> agent_paths;
        unordered_map<int, vector<pair<int, int>>> agent_paths_new;


        for (auto& [agent_id, positions] : agents) {
            int collision_avoidance_count = 0;
            auto path = randomized_astar(positions.first, positions.second, grid, reserved, collision_avoidance_count);
            total_collision_avoidance_count += collision_avoidance_count;

            if (path.empty()) {
                stringstream error_msg;
                error_msg << "Agent " << agent_id << " の経路が見つかりませんでした。";
                cout << error_msg.str() << endl;
                output_lines.push_back(error_msg.str());
                continue;
            }
            agent_paths[agent_id] = path;
            // 予約済み位置の更新
            for (size_t t = 0; t < path.size(); ++t) {
                reserved[t].insert(path[t]);
            }

            auto path_new = randomized_astar_new(positions.first, positions.second, grid);
            if (path_new.empty()) {
                stringstream error_msg;
                error_msg << "Agent " << agent_id << " の経路が見つかりませんでした（新しい関数）。";
                cout << error_msg.str() << endl;
                output_lines.push_back(error_msg.str());
                continue;
            }
            agent_paths_new[agent_id] = path_new;
        }

        all_agent_paths_runs.push_back(agent_paths);
        all_agent_paths_runs_new.push_back(agent_paths_new);

    }

    cout << "全エージェントの衝突を回避した回数の合計: " << total_collision_avoidance_count << endl;


    // セグメントの利用効率（通過確率の分布）の計算
    auto calculate_segment_efficiency = [](const vector<unordered_map<int, vector<pair<int, int>>>>& all_paths) {
        unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash> segment_counts;
        unordered_map<int, unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash>> agent_normalized_scores;

        // セグメントの通過確率の計算
        for (auto& agent_paths : all_paths) {
            for (auto& [agent_id, path] : agent_paths) {
                for (size_t t = 0; t < path.size() - 1; ++t) {
                    auto segment = make_pair(
                        make_tuple(path[t].first, path[t].second, t),
                        make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                    );
                    segment_counts[segment]++;
                }
            }
        }

        // エージェントごとにセグメントの通過確率を正規化
        for (auto& agent_paths : all_paths) {
            for (auto& [agent_id, path] : agent_paths) {
                unordered_map<int, double> timestep_sums;

                // タイムステップごとのスコア合計を計算
                for (size_t t = 0; t < path.size() - 1; ++t) {
                    auto segment = make_pair(
                        make_tuple(path[t].first, path[t].second, t),
                        make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                    );
                    timestep_sums[t] += segment_counts[segment];
                }

                // セグメントの通過確率の正規化
                for (size_t t = 0; t < path.size() - 1; ++t) {
                    auto segment = make_pair(
                        make_tuple(path[t].first, path[t].second, t),
                        make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                    );
                    double sum = timestep_sums[t];
                    agent_normalized_scores[agent_id][segment] =
                        (sum > 0) ? (segment_counts[segment] / sum) : 0.0;
                }
            }
        }

        // 全エージェントのセグメントを集計
        unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash> segment_total_probabilities;
        for (auto& agent_scores : agent_normalized_scores) {
            for (auto& [segment, score] : agent_scores.second) {
                segment_total_probabilities[segment] += score;
            }
        }

        // 分散と標準偏差の計算
        double total_segments = segment_total_probabilities.size();
        double total_score = 0;
        for (auto& [segment, score] : segment_total_probabilities) {
            total_score += score;
        }

        double mean_score = total_score / total_segments;

        double variance = 0;
        for (auto& [segment, score] : segment_total_probabilities) {
            variance += (score - mean_score) * (score - mean_score);
        }
        variance /= total_segments;

        double standard_deviation = sqrt(variance);

        cout << "セグメントのの分散: " << variance << endl;
        cout << "セグメントの通過確率の標準偏差: " << standard_deviation << endl;
    };

    cout << "\n衝突回避ありのセグメント利用効率:" << endl;
    calculate_segment_efficiency(all_agent_paths_runs);

    cout << "\n衝突回避なしのセグメント利用効率:" << endl;
    calculate_segment_efficiency(all_agent_paths_runs_new);





    //k匿名化
    unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash> segment_counts;
    unordered_map<int, unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash>> agent_normalized_scores;
    unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash> segment_k_values;

    // セグメントの通過確率の計算
    for (auto& agent_paths : all_agent_paths_runs) {
        for (auto& [agent_id, path] : agent_paths) {
            for (size_t t = 0; t < path.size() - 1; ++t) {
                auto segment = make_pair(
                    make_tuple(path[t].first, path[t].second, t),
                    make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                );
                segment_counts[segment]++;
            }
        }
    }

    // エージェントごとにセグメントの通過確率を正規化
    for (auto& agent_paths : all_agent_paths_runs) {
        for (auto& [agent_id, path] : agent_paths) {
            unordered_map<int, double> timestep_sums;

            // タイムステップごとのスコア合計を計算
            for (size_t t = 0; t < path.size() - 1; ++t) {
                auto segment = make_pair(
                    make_tuple(path[t].first, path[t].second, t),
                    make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                );
                timestep_sums[t]++;
            }

            // セグメントの通過確率の正規化
            for (size_t t = 0; t < path.size() - 1; ++t) {
                auto segment = make_pair(
                    make_tuple(path[t].first, path[t].second, t),
                    make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                );
                double sum = timestep_sums[t];
                agent_normalized_scores[agent_id][segment] =
                    (sum > 0) ? (segment_counts[segment] / sum) : 0.0;
            }
        }
    }

    // 全エージェントのk値を算出
    unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double, PairHash> segment_total_probabilities;
    int total_segment_count = 0;  // 全セグメント数（重複カウント）

    for (auto& agent_paths : all_agent_paths_runs) {
        for (auto& [agent_id, path] : agent_paths) {
            for (size_t t = 0; t + 1 < path.size(); ++t) {
                auto segment = make_pair(
                    make_tuple(path[t].first, path[t].second, t),
                    make_tuple(path[t + 1].first, path[t + 1].second, t + 1)
                );
                // セグメントごとの最大通過確率を計算
                segment_total_probabilities[segment] += agent_normalized_scores[agent_id][segment];

                // 全セグメント数をカウント（重複を含む）
                total_segment_count++;
            }
        }
    }

    // データ残留率の計算と標準出力への出力
    for (double k_value : k_values) {
        int retained_segment_count = 0;

        for (auto& [segment, max_probability] : segment_total_probabilities) {
            // k値を超えるセグメント数をカウント
            if (max_probability >= k_value) {
                retained_segment_count++;
            }
        }

        // データ残留率の計算
        double retention_rate = (static_cast<double>(retained_segment_count) / total_segment_count) * 100.0;

        stringstream k_anonymization_output;
        k_anonymization_output << "k = " << k_value << ": Data Retention Rate = "
                            << fixed << setprecision(2) << retention_rate << "%";
        cout << k_anonymization_output.str() << endl;
        output_lines.push_back(k_anonymization_output.str());
    }


    //     // デバッグ: 正規化スコアの出力
    // for (auto& [segment, probability] : segment_max_probabilities) {
    //     cout << "Segment: (" << get<0>(segment.first) << "," << get<1>(segment.first) << "," << get<2>(segment.first)
    //         << ") -> (" << get<0>(segment.second) << "," << get<1>(segment.second) << "," << get<2>(segment.second)
    //         << "), Max Probability: " << probability << endl;
    // }


    auto total_end_time = chrono::steady_clock::now();
    cout << "Total time: "
         << chrono::duration_cast<chrono::seconds>(total_end_time - total_start_time).count()
         << " seconds" << endl;

    return 0;
}
