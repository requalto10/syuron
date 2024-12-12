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
#include <functional>

using namespace std;

// グリッドサイズとエージェント数の設定
const int n = 100;
const int m = 100;
const int a = 200;

// パラメータの設定
vector<int> k_values = {2, 3, 4, 5, 10, 15, 20}; // スコアの閾値のリスト
const int time_limit = 200;   // Time = 0 ~ 200 の範囲のみを描画
const int num_solve = 1000;     // 経路探索の実行回数
const int d = 7;              // 距離の閾値
const double constant = 1.0;  // スコア計算時の定数

// 乱数生成器の設定
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> dis_noise(0.0, 10.0);

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

// ランダム性を加えた A* アルゴリズムの実装
vector<pair<int, int>> randomized_astar(pair<int, int> start, pair<int, int> goal,
                                        vector<vector<int>>& grid,
                                        unordered_map<int, unordered_set<pair<int, int>>>& occupied) {
    // 優先度付きキュー（小さい順）
    using Node = tuple<double, int, pair<int, int>, vector<pair<int, int>>>;
    priority_queue<Node, vector<Node>, greater<Node>> open_set;

    double noise = dis_noise(gen);
    open_set.push(make_tuple(heuristic(start, goal) + noise, 0, start, vector<pair<int, int>>{start}));
    unordered_set<pair<pair<int, int>, int>> closed_set;

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
    // 処理時間計測開始
    auto total_start_time = chrono::steady_clock::now();

    // 標準出力とパラメータ情報を保存するリスト
    vector<string> output_lines;
    vector<string> detail_output_lines;  // 詳細情報を保存するリスト

    // エージェントの始点と終点をランダムに設定（全ての実行で共通）
    vector<pair<int, int>> all_positions;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < m; ++y) {
            all_positions.push_back(make_pair(x, y));
        }
    }
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

    // エージェントの情報を詳細出力に保存
    cout << "Agent Start and Goal Positions:" << endl;
    detail_output_lines.push_back("Agent Start and Goal Positions:");
    for (int i = 0; i < a; ++i) {
        stringstream agent_info;
        agent_info << "Agent " << i + 1 << ": Start (" << agent_starts[i].first << ", " << agent_starts[i].second
                   << "), Goal (" << agent_goals[i].first << ", " << agent_goals[i].second << ")";
        cout << agent_info.str() << endl;
        detail_output_lines.push_back(agent_info.str());
    }

    // エージェント情報のマップ
    unordered_map<int, pair<pair<int, int>, pair<int, int>>> agents;
    for (int i = 0; i < a; ++i) {
        agents[i + 1] = make_pair(agent_starts[i], agent_goals[i]);
    }

    // グリッドの初期化
    vector<vector<int>> grid(m, vector<int>(n, 0));

    // 全実行の経路情報を保存するベクター
    vector<unordered_map<int, vector<pair<int, int>>>> all_agent_paths_runs;

    // 経路探索パートの処理時間計測開始
    auto pathfinding_start = chrono::steady_clock::now();

    // 複数回の経路探索を実行
    for (int run = 0; run < num_solve; ++run) {
        if ( run % 100 == 0) {
            stringstream run_info;
            run_info << "\n--- Run " << run + 1 << " ---";
            cout << run_info.str() << endl;
            detail_output_lines.push_back(run_info.str());
        }

        // 予約済みの位置を記録するマップ
        unordered_map<int, unordered_set<pair<int, int>>> reserved;

        // 各エージェントの経路を記録するマップ
        unordered_map<int, vector<pair<int, int>>> agent_paths;
        int max_path_length_run = 0;

        // 各エージェントの経路を計算
        for (auto& [agent_id, positions] : agents) {
            auto path = randomized_astar(positions.first, positions.second, grid, reserved);
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
            if (path.size() > max_path_length_run) {
                max_path_length_run = path.size();
            }
        }

        // エージェントの経路を保存
        all_agent_paths_runs.push_back(agent_paths);
    }

    // 経路探索パートの処理時間計測終了
    auto pathfinding_end = chrono::steady_clock::now();
    auto pathfinding_time = chrono::duration_cast<chrono::milliseconds>(pathfinding_end - pathfinding_start).count();

    // ここからは最後の実行結果を使用して可視化と分析を行う

    // 匿名化処理パートの処理時間計測開始
    auto anonymization_start = chrono::steady_clock::now();

    // 最後の実行結果を取得
    auto agent_paths = all_agent_paths_runs.back();

    // max_path_length を再計算
    int max_path_length = 0;
    for (const auto& [agent_id, path] : agent_paths) {
        if (path.size() > max_path_length) {
            max_path_length = path.size();
        }
    }

    // シミュレーションの開始
    unordered_map<int, vector<tuple<int, int, int>>> agent_time_positions;
    for (const auto& [agent_id, _] : agents) {
        agent_time_positions[agent_id] = vector<tuple<int, int, int>>();
    }

    int max_time = min(max_path_length, time_limit);

    for (int time_step = 0; time_step <= max_time; ++time_step) {
        // 各エージェントの位置を更新
        for (const auto& [agent_id, path] : agent_paths) {
            int x, y;
            if (time_step < path.size()) {
                x = path[time_step].first;
                y = path[time_step].second;
            } else {
                // 目的地に到達済み
                x = path.back().first;
                y = path.back().second;
            }
            agent_time_positions[agent_id].push_back(make_tuple(time_step, x, y));
        }

        // 進捗状況を一定の間隔で表示
        if (time_step % 50 == 0) {
            stringstream progress_msg;
            progress_msg << "Time Step " << time_step << " / " << max_time;
            cout << progress_msg.str() << endl;
            detail_output_lines.push_back(progress_msg.str());
        }
    }

    // 全てのセグメントを収集
    vector<pair<tuple<int, int, int>, tuple<int, int, int>>> all_segments;

    for (const auto& [agent_id, positions] : agent_time_positions) {
        for (size_t i = 0; i < positions.size() - 1; ++i) {
            int t1, x1, y1;
            tie(t1, x1, y1) = positions[i];
            int t2, x2, y2;
            tie(t2, x2, y2) = positions[i + 1];
            all_segments.push_back(make_pair(make_tuple(x1, y1, t1), make_tuple(x2, y2, t2)));
        }
    }

    // セグメントの通過エージェント数をカウント
    unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, int> segment_counts;
    for (const auto& seg : all_segments) {
        segment_counts[seg]++;
    }

    // セグメントを空間・時間でインデックス化
    unordered_map<tuple<int, int, int>, vector<pair<tuple<int, int, int>, tuple<int, int, int>>>> segment_buckets;
    int bucket_size = d;  // バケットのサイズを距離の閾値に合わせる

    for (const auto& seg : all_segments) {
        int x1, y1, t1;
        tie(x1, y1, t1) = seg.first;
        auto key = make_tuple(x1 / bucket_size, y1 / bucket_size, t1 / bucket_size);
        segment_buckets[key].push_back(seg);
    }

    // 全ての k の値についてデータ残留率を計算
    unordered_map<int, double> data_residual_rates;

    for (auto k : k_values) {
        unordered_map<pair<tuple<int, int, int>, tuple<int, int, int>>, double> segment_scores;
        set<pair<tuple<int, int, int>, tuple<int, int, int>>> processed_segments;

        for (const auto& seg : all_segments) {
            if (processed_segments.find(seg) != processed_segments.end()) {
                continue;
            }
            processed_segments.insert(seg);
            double score = 0.0;
            int x1_a, y1_a, t1_a;
            tie(x1_a, y1_a, t1_a) = seg.first;
            auto key = make_tuple(x1_a / bucket_size, y1_a / bucket_size, t1_a / bucket_size);
            // 自分と近傍のバケットのキーを生成
            vector<tuple<int, int, int>> neighbor_keys;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dt = -1; dt <= 1; ++dt) {
                        neighbor_keys.push_back(make_tuple(get<0>(key) + dx, get<1>(key) + dy, get<2>(key) + dt));
                    }
                }
            }
            // 近傍のセグメントのみを探索
            for (const auto& neighbor_key : neighbor_keys) {
                auto it = segment_buckets.find(neighbor_key);
                if (it != segment_buckets.end()) {
                    for (const auto& seg_b : it->second) {
                        if (seg_b == seg) continue;
                        int x1_b, y1_b, t1_b;
                        tie(x1_b, y1_b, t1_b) = seg_b.first;
                        // 距離を計算（マンハッタン距離）
                        double space_time_distance = abs(x1_a - x1_b) + abs(y1_a - y1_b) + abs(t1_a - t1_b);
                        if (space_time_distance <= d) {
                            // スコアを加算（変更箇所）
                            double distance_squared = space_time_distance * space_time_distance + 1e-6;
                            score += constant * segment_counts[seg_b] / distance_squared;
                        }
                    }
                }
            }
            if (score >= k) {
                segment_scores[seg] = score;
            }
        }

        // 匿名化後に公開できるセグメント数を計算
        int num_anony_seg = segment_scores.size();

        // 全ての経路に含まれるセグメント数を計算
        int num_all_seg = all_segments.size();

        // データ残留率を計算
        double data_residual_rate = num_all_seg > 0 ? 100.0 * num_anony_seg / num_all_seg : 0.0;
        data_residual_rates[k] = data_residual_rate;

        // 結果を出力
        stringstream result_msg;
        result_msg << "\nFor k = " << k << ":";
        result_msg << "\nTotal number of segments (num_all_seg): " << num_all_seg;
        result_msg << "\nNumber of anonymous segments (num_anony_seg): " << num_anony_seg;
        result_msg << fixed << setprecision(4);
        result_msg << "\nData residual rate: " << data_residual_rate << "%\n";
        cout << result_msg.str();
        output_lines.push_back(result_msg.str());
    }

    // 匿名化処理パートの処理時間計測終了
    auto anonymization_end = chrono::steady_clock::now();
    auto anonymization_time = chrono::duration_cast<chrono::milliseconds>(anonymization_end - anonymization_start).count();

    // 処理時間計測終了
    auto total_end_time = chrono::steady_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(total_end_time - total_start_time).count();

    // 各処理時間を出力
    cout << "\n経路探索の処理時間: " << pathfinding_time / 1000.0 << " 秒" << endl;
    cout << "匿名化処理の処理時間: " << anonymization_time / 1000.0 << " 秒" << endl;
    cout << "\n合計処理時間: " << total_time / 1000.0 << " 秒" << endl;

    // 処理時間の出力をファイルに保存
    output_lines.push_back("\n経路探索の処理時間: " + to_string(pathfinding_time / 1000.0) + " 秒");
    output_lines.push_back("匿名化処理の処理時間: " + to_string(anonymization_time / 1000.0) + " 秒");
    output_lines.push_back("\n合計処理時間: " + to_string(total_time / 1000.0) + " 秒");

    // 標準出力の内容をファイルに保存（ファイル名を日時に変更）
    string timestamp = getTimestamp();
    ofstream outfile(timestamp + ".txt");
    for (const auto& line : output_lines) {
        outfile << line << endl;
    }
    // 詳細情報を最後に追記
    for (const auto& line : detail_output_lines) {
        outfile << line << endl;
    }
    outfile.close();

    // 経路情報をファイルに保存（最後の実行結果）
    ofstream pathfile("agent_paths_limited.txt");
    pathfile << "agent_id,x,y,time_step\n";
    for (const auto& [agent_id, positions] : agent_time_positions) {
        for (const auto& [time_step, x, y] : positions) {
            pathfile << agent_id << "," << x << "," << y << "," << time_step << "\n";
        }
    }
    pathfile.close();

    return 0;
}
