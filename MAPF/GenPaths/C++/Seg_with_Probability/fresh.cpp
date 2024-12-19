#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <random>
#include <functional>
#include <ctime>
#include <limits>
#include <cassert>
#include <string>

using namespace std;

struct Position {
    int x,y;
};

struct Segment {
    int x1,y1,x2,y2;
    double cx, cy;
    int time_step;
    bool operator==(const Segment &o) const {
        return x1==o.x1 && y1==o.y1 && x2==o.x2 && y2==o.y2 && time_step == o.time_step;
    }
};

struct SegmentHash {
    std::size_t operator()(const Segment &s) const {
        std::size_t h = 0;
        h ^= std::hash<int>()(s.x1) + 0x9e3779b97f4a7c16ULL;
        h ^= std::hash<int>()(s.y1) + 0x9e3779b97f4a7c16ULL;
        h ^= std::hash<int>()(s.x2) + 0x9e3779b97f4a7c16ULL;
        h ^= std::hash<int>()(s.y2) + 0x9e3779b97f4a7c16ULL;
        h ^= std::hash<int>()(s.time_step) + 0x9e3779b97f4a7c16ULL;
        return h;
    }
};

struct Node {
    int x,y;
    double g,h;
    Node *parent;
};

struct ConflictConstraint {
    std::map<int, std::set<pair<int,int>>> not_allowed;
};

struct Path {
    vector<Position> nodes;
};

// グローバルパラメータ
static int m; 
static int a; 
static int num_solve;
static int d; 
static vector<double> k_values; 
static vector<Position> starts, goals; 
static int collision_avoidance_count_with = 0; 
static std::mt19937_64 rng((unsigned long)time(NULL));

// ヒューリスティック用乱数範囲: 0 ~ m/5
// 後で使用するため関数外に保持
static double noise_max; 
static uniform_real_distribution<double> dist; 

bool is_free(int x,int y) {
    return (0 <= x && x < m && 0 <= y && y < m);
}

double heuristic(int x,int y,int gx,int gy) {
    double base = abs(x - gx) + abs(y - gy);
    // 0 ~ (m/5)の範囲で乱数生成
    double noise = dist(rng);
    return base + noise;
}

Path a_star_single_agent(int sx,int sy,int gx,int gy,
                         const ConflictConstraint &constraints) 
{
    typedef tuple<double,int,int,double,int,int> PQItem; 
    priority_queue<PQItem, vector<PQItem>, greater<PQItem>> pq;
    vector<vector<double>> gScore(m,vector<double>(m,1e9));
    vector<vector<Position>> parent(m,vector<Position>(m,{-1,-1}));
    double h0 = heuristic(sx,sy,gx,gy);
    gScore[sx][sy]=0.0;
    pq.push({h0+0.0,sx,sy,0.0,-1,-1});

    while(!pq.empty()){
        auto [f,x,y,gc,px,py] = pq.top(); pq.pop();
        if(x==gx && y==gy) {
            int length = (int)round(gc);
            Path path;
            path.nodes.resize((size_t)length+1);
            int cx=x, cy=y;
            for(int i=length;i>=0;i--){
                if(cx<0||cx>=m||cy<0||cy>=m) {
                    path.nodes.clear();
                    return path;
                }
                path.nodes[(size_t)i] = {cx,cy};
                int nx=parent[cx][cy].x, ny=parent[cx][cy].y;
                cx=nx;cy=ny;
            }
            return path;
        }

        if(gc>gScore[x][y]) continue;

        vector<pair<int,int>> dirs = {{1,0},{-1,0},{0,1},{0,-1}};
        std::shuffle(dirs.begin(),dirs.end(),rng);

        for(auto &dxy: dirs){
            int nx=x+dxy.first, ny=y+dxy.second;
            if(!is_free(nx,ny)) continue;
            double ng=gc+1.0;
            int t_step = (int)round(ng);
            auto it = constraints.not_allowed.find(t_step);
            if(it!=constraints.not_allowed.end()){
                if(it->second.count({nx,ny})) {
                    continue;
                }
            }

            if(ng<gScore[nx][ny]){
                gScore[nx][ny]=ng;
                double nh=heuristic(nx,ny,gx,gy);
                parent[nx][ny]={x,y};
                pq.push({ng+nh,nx,ny,ng,x,y});
            }
        }
    }

    Path empty_path;
    return empty_path;
}

vector<Path> solve_mapf_with_collision_avoidance() {
    vector<Path> solutions(a);
    ConflictConstraint constraints;

    for(int i=0;i<a;i++){
        int trial=0;
        bool found_path = false;
        while(trial<1000){
            trial++;
            Path p = a_star_single_agent(starts[i].x, starts[i].y, goals[i].x, goals[i].y, constraints);
            if(p.nodes.empty()){
                continue;
            }
            bool collision_found = false;
            for(int j=0;j<i;j++){
                const Path &op = solutions[j];
                if(op.nodes.empty() || p.nodes.empty()) {
                    continue;
                }

                int maxT = (int)max(p.nodes.size(), op.nodes.size());
                for(int t=0; t<maxT; t++){
                    int tx, ty, ox, oy;

                    if(t<(int)p.nodes.size()){
                        tx=p.nodes[t].x; ty=p.nodes[t].y;
                    } else {
                        tx=p.nodes.back().x; ty=p.nodes.back().y;
                    }

                    if(t<(int)op.nodes.size()){
                        ox=op.nodes[t].x; oy=op.nodes[t].y;
                    } else {
                        ox=op.nodes.back().x; oy=op.nodes.back().y;
                    }

                    if(tx==ox && ty==oy) {
                        collision_found = true;
                        collision_avoidance_count_with++;
                        constraints.not_allowed[t].insert({tx,ty});
                    }
                }
            }
            if(!collision_found){
                solutions[i]=p;
                for(int t=0; t<(int)p.nodes.size(); t++){
                    constraints.not_allowed[t].insert({p.nodes[t].x, p.nodes[t].y});
                }
                found_path = true;
                break;
            }
        }
        if(!found_path){
            return solutions;
        }
    }

    return solutions;
}

vector<Path> solve_mapf_without_collision_avoidance() {
    ConflictConstraint no_constraints;
    vector<Path> solutions(a);
    for(int i=0;i<a;i++){
        Path p = a_star_single_agent(starts[i].x, starts[i].y, goals[i].x, goals[i].y, no_constraints);
        solutions[i] = p;
    }
    return solutions;
}

static vector<vector<Path>> run_solutions(bool with_collision) {
    vector<vector<Path>> solutions_set(num_solve);
    for(int i=0;i<num_solve;i++){
        // 進捗状況出力（標準エラー出力）
        cerr << (with_collision?"[With Collision]":"[Without Collision]") 
             << " Solving " << i+1 << "/" << num_solve << "...\n";

        if(with_collision) {
            solutions_set[i] = solve_mapf_with_collision_avoidance();
        } else {
            solutions_set[i] = solve_mapf_without_collision_avoidance();
        }
    }
    return solutions_set;
}

struct AgentSolutionSegments {
    vector<vector<vector<Segment>>> agent_segments;
    unordered_set<Segment,SegmentHash> all_segments_set;
};

static double segment_distance(const Segment &s1, const Segment &s2) {
    double dx = s1.cx - s2.cx;
    double dy = s1.cy - s2.cy;
    return sqrt(dx*dx + dy*dy);
}

AgentSolutionSegments extract_segments(const vector<vector<Path>> &solutions_set) {
    AgentSolutionSegments result;
    result.agent_segments.resize(a);

    for(int ag=0; ag<a; ag++){
        result.agent_segments[ag].resize(solutions_set.size());
        for(int si=0; si<(int)solutions_set.size(); si++){
            const Path &P = solutions_set[si][ag];
            for(int t=0; t+1<(int)P.nodes.size(); t++){
                Segment seg;
                seg.x1 = P.nodes[t].x;
                seg.y1 = P.nodes[t].y;
                seg.x2 = P.nodes[t+1].x;
                seg.y2 = P.nodes[t+1].y;
                seg.time_step = t;
                seg.cx=(seg.x1+seg.x2)/2.0;
                seg.cy=(seg.y1+seg.y2)/2.0;
                result.agent_segments[ag][si].push_back(seg);
            }
        }
    }

    for(int ag=0; ag<a; ag++){
        for(int si=0; si<(int)result.agent_segments[ag].size(); si++){
            for(auto &seg: result.agent_segments[ag][si]) {
                result.all_segments_set.insert(seg);
            }
        }
    }
    return result;
}

static vector<Segment> to_vector(const unordered_set<Segment,SegmentHash> &ss) {
    vector<Segment> v(ss.begin(), ss.end());
    return v;
}

static vector<vector<double>> compute_pass_probabilities(
    const vector<vector<Path>> &solutions_set,
    const AgentSolutionSegments &ass
) {
    vector<Segment> all_segments_vec = to_vector(ass.all_segments_set);
    if(all_segments_vec.empty()) {
        return vector<vector<double>>(a, vector<double>(0));
    }

    unordered_map<Segment,int,SegmentHash> seg_to_id;
    for(int i=0;i<(int)all_segments_vec.size();i++){
        seg_to_id[all_segments_vec[i]]=i;
    }

    vector<vector<int>> count(a,vector<int>((int)all_segments_vec.size(),0));

    for(int ag=0;ag<a;ag++){
        for(int si=0; si<(int)solutions_set.size(); si++){
            for(auto &seg: ass.agent_segments[ag][si]){
                int id=seg_to_id[seg];
                if(id>=0 && id<(int)all_segments_vec.size()) {
                    count[ag][id]++;
                }
            }
        }
    }

    vector<vector<pair<int,double>>> neighbors(all_segments_vec.size());
    for(int i=0;i<(int)all_segments_vec.size();i++){
        for(int j=i+1;j<(int)all_segments_vec.size();j++){
            double dist = segment_distance(all_segments_vec[i], all_segments_vec[j]);
            if(dist <= d) {
                neighbors[i].push_back({j,dist});
                neighbors[j].push_back({i,dist});
            }
        }
    }

    vector<vector<double>> score(a, vector<double>((int)all_segments_vec.size(),0.0));
    for(int ag=0;ag<a;ag++){
        for(int sid=0; sid<(int)all_segments_vec.size(); sid++){
            double base = (double)count[ag][sid];
            double add = 0.0;
            if(base>0.0) {
                for(auto &nb: neighbors[sid]) {
                    int osid=nb.first;
                    double dist=nb.second;
                    double other_score = (double)count[ag][osid];
                    if(other_score>0.0 && dist>0.0) {
                        add += other_score*(1.0/(dist*dist));
                    }
                }
            }
            score[ag][sid] = base + add;
        }
    }

    vector<vector<double>> probability(a,vector<double>((int)all_segments_vec.size(),0.0));

    int max_t_all=0;
    for (auto &seg: all_segments_vec) {
        if(seg.time_step > max_t_all) max_t_all=seg.time_step;
    }

    for(int ag=0;ag<a;ag++){
        vector<double> timestep_sum(max_t_all+1,0.0);
        for(int sid=0; sid<(int)all_segments_vec.size(); sid++){
            if(score[ag][sid]>0.0){
                int t=all_segments_vec[sid].time_step;
                if(t>=0 && t<=max_t_all)
                    timestep_sum[t]+=score[ag][sid];
            }
        }
        for(int sid=0; sid<(int)all_segments_vec.size(); sid++){
            if(score[ag][sid]>0.0){
                int t=all_segments_vec[sid].time_step;
                if(t>=0 && t<=max_t_all && timestep_sum[t]>0) {
                    probability[ag][sid] = score[ag][sid]/timestep_sum[t];
                }
            }
        }
    }

    return probability;
}


static pair<double,double> compute_variance_std_of_kvalues(
    const vector<vector<double>> &probability
) {
    if(probability.empty()) {
        return {0.0, 0.0};
    }
    int a = (int)probability.size();
    int n_seg = 0;
    for (int ag=0; ag<a; ag++){
        n_seg = max(n_seg, (int)probability[ag].size());
    }
    if(n_seg==0) return {0.0, 0.0};

    vector<double> k_values_of_segments(n_seg,0.0);
    for(int sid=0; sid<n_seg; sid++){
        double sum_p=0.0;
        for(int ag=0; ag<a; ag++){
            if(sid<(int)probability[ag].size()) {
                sum_p += probability[ag][sid];
            }
        }
        k_values_of_segments[sid]=sum_p;
    }

    double mean=0.0;
    for(auto val: k_values_of_segments) mean+=val;
    mean /= (double)k_values_of_segments.size();

    double var=0.0;
    for(auto val:k_values_of_segments){
        double diff=val-mean;
        var+=diff*diff;
    }
    var/=(double)k_values_of_segments.size();
    double stddev=sqrt(var);

    return {var,stddev};
}


static vector<double> compute_data_retention(
    const vector<vector<double>> &probability,
    const vector<Segment> &all_segments_vec
) {
    int n_seg = (int)all_segments_vec.size();
    vector<double> ret;
    if(n_seg==0) {
        for(auto kv: k_values) {
            (void)kv; 
            ret.push_back(0.0);
        }
        return ret;
    }

    vector<double> k_value_of_segment(n_seg,0.0);
    for(int sid=0; sid<n_seg; sid++){
        double sum_p=0.0;
        for(int ag=0;ag<(int)probability.size();ag++){
            if(sid<(int)probability[ag].size())
                sum_p += probability[ag][sid];
        }
        k_value_of_segment[sid]=sum_p;
    }

    for(auto kv: k_values){
        int count_over=0;
        for(int sid=0; sid<n_seg; sid++){
            if(k_value_of_segment[sid]>kv){
                count_over++;
            }
        }
        double retention = 0.0;
        if(n_seg>0) retention = ((double)count_over/(double)n_seg)*100.0; 
        ret.push_back(retention);
    }
    return ret;
}


int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // パラメータ例(任意に変更可)
    m = 60;
    a = 400;
    num_solve = 200;
    d = 5;
    k_values = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2};

    // ノイズ範囲設定: 0 ~ m/5
    noise_max = m/5.0; 
    dist = uniform_real_distribution<double>(0.0, noise_max);

    string filename = "m" + to_string(m) 
                    + "_a" + to_string(a) 
                    + "_num" + to_string(num_solve) 
                    + "_d" + to_string(d) 
                    + ".txt";
    freopen(filename.c_str(), "w", stdout);

    // 標準エラー出力で進捗表示
    cerr << "Starting setup...\n";

    starts.resize(a);
    goals.resize(a);
    for(int i=0;i<a;i++){
        starts[i] = { (int)(rng()%m), (int)(rng()%m) };
        goals[i] = { (int)(rng()%m), (int)(rng()%m) };
    }

    cerr << "Running solutions with collision avoidance...\n";
    vector<vector<Path>> solutions_with = run_solutions(true);

    cerr << "Running solutions without collision avoidance...\n";
    vector<vector<Path>> solutions_without = run_solutions(false);

    cerr << "Extracting segments...\n";
    AgentSolutionSegments ass_with = extract_segments(solutions_with);
    AgentSolutionSegments ass_without = extract_segments(solutions_without);

    cerr << "Computing probabilities...\n";
    vector<vector<double>> probability_with = compute_pass_probabilities(solutions_with, ass_with);
    vector<vector<double>> probability_without = compute_pass_probabilities(solutions_without, ass_without);

    vector<Segment> all_segments_with = to_vector(ass_with.all_segments_set);
    vector<Segment> all_segments_without = to_vector(ass_without.all_segments_set);

    cerr << "Computing data retention...\n";
    vector<double> data_retention_with = compute_data_retention(probability_with, all_segments_with);
    vector<double> data_retention_without = compute_data_retention(probability_without, all_segments_without);

    cerr << "Computing variance and std of k-values...\n";
    auto [var_with, std_with] = compute_variance_std_of_kvalues(probability_with);
    auto [var_without, std_without] = compute_variance_std_of_kvalues(probability_without);

    cerr << "All computations done. Writing results to file...\n";

    // 以下、出力
    cout << "【パラメータ情報】\n";
    cout << "m = " << m << "\n";
    cout << "a = " << a << "\n";
    cout << "num_solve = " << num_solve << "\n";
    cout << "d = " << d << "\n";

    cout << "\nk_values(閾値) =";
    for (auto kv : k_values) {
        cout << " " << kv;
    }
    cout << "\n";

    cout << "\n【結果出力】\n";
    cout << "■ 衝突回避あり探索で発生した衝突回避回数: " << collision_avoidance_count_with << "\n\n";

    cout << "■ データ残留率(%)について:\n";
    cout << "   k_values:";
    for(auto kv:k_values) cout << " " << kv;
    cout << "\n\n";

    cout << "   (衝突回避ありの場合)\n";
    cout << "   データ残留率(%):";
    for(auto v:data_retention_with) cout<<" "<<v;
    cout << "\n\n";

    cout << "   (衝突回避なしの場合)\n";
    cout << "   データ残留率(%):";
    for(auto v:data_retention_without) cout<<" "<<v;
    cout << "\n\n";

    cout << "■ セグメントk値の分散・標準偏差:\n";
    cout << "   (衝突回避あり) 分散=" << var_with << ", 標準偏差=" << std_with << "\n";
    cout << "   (衝突回避なし) 分散=" << var_without << ", 標準偏差=" << std_without << "\n";

    cerr << "Done.\n";

    return 0;
}
