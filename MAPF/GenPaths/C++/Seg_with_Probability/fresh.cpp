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
        // Hash組み合わせは簡易的
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
static int m; // グリッドサイズm×m
static int a; // エージェント数
static int num_solve; // 各設定での経路探索回数
static int d; // セグメント影響範囲
static vector<double> k_values; // k閾値

static vector<Position> starts, goals; 
static int collision_avoidance_count_with = 0;
static std::mt19937_64 rng((unsigned long)time(NULL));

bool is_free(int x,int y) {
    return (0 <= x && x < m && 0 <= y && y < m);
}

double heuristic(int x,int y,int gx,int gy) {
    double base = abs(x - gx) + abs(y - gy);
    std::uniform_real_distribution<double> dist(0.0,1.0);
    return base + dist(rng);
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
                    // 不正状態はスキップ
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

// 衝突回避ありMAPF（簡易実装）
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
                // 経路見つからず
                continue;
            }
            // 衝突チェック
            bool collision_found = false;
            for(int j=0;j<i;j++){
                const Path &op = solutions[j];
                if(op.nodes.empty() || p.nodes.empty()) {
                    // 空経路なら衝突しようがないが、空経路はありえないはず
                    continue;
                }

                int maxT = (int)max(p.nodes.size(), op.nodes.size());
                for(int t=0; t<maxT; t++){
                    int tx, ty, ox, oy;
                    if(p.nodes.empty()) break; // 念のため
                    if(op.nodes.empty()) break; 
                    
                    // pのt番目または最後
                    if(t<(int)p.nodes.size()){
                        tx=p.nodes[t].x; ty=p.nodes[t].y;
                    } else {
                        tx=p.nodes.back().x; ty=p.nodes.back().y;
                    }
                    // opのt番目または最後
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
                // 衝突なし
                solutions[i]=p;
                for(int t=0; t<(int)p.nodes.size(); t++){
                    constraints.not_allowed[t].insert({p.nodes[t].x, p.nodes[t].y});
                }
                found_path = true;
                break;
            }
        }
        if(!found_path){
            // 経路が見つからなかった場合、このエージェントから先は空経路
            // 全体的に満足行く解がないが、ひとまず返す
            // 実務的にはここで再試行やパラメータ変更を検討すべき
            return solutions;
        }
    }

    return solutions;
}

// 衝突回避なしMAPF
vector<Path> solve_mapf_without_collision_avoidance() {
    ConflictConstraint no_constraints;
    vector<Path> solutions(a);
    for(int i=0;i<a;i++){
        Path p = a_star_single_agent(starts[i].x, starts[i].y, goals[i].x, goals[i].y, no_constraints);
        // pがemptyならそのまま続行するが、この場合も後続処理でセグメントがなくてもOKなようにする
        solutions[i] = p;
    }
    return solutions;
}

static vector<vector<Path>> run_solutions(bool with_collision) {
    vector<vector<Path>> solutions_set(num_solve);
    for(int i=0;i<num_solve;i++){
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
            // Pが空または1ノードしかない場合はセグメントなし
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
        // セグメントがない場合、確率計算は全て0
        return vector<vector<double>>(a, vector<double>(0));
    }

    unordered_map<Segment,int,SegmentHash> seg_to_id;
    for(int i=0;i<(int)all_segments_vec.size();i++){
        seg_to_id[all_segments_vec[i]]=i;
    }

    vector<vector<int>> count(a,vector<int>((int)all_segments_vec.size(),0));
    // agentが通ったseg_idを記録する必要が特にないので削除可能

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

    // time_stepごとの正規化
    // agentごとに最大time_step検索
    // score>0のセグメントのみ正規化対象
    // 1時間ステップ内で合計が1になるよう正規化
    // time_stepはsegに格納
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

static vector<double> compute_data_retention(
    const vector<vector<double>> &probability,
    const vector<Segment> &all_segments_vec
) {
    int n_seg = (int)all_segments_vec.size();
    vector<double> ret;
    if(n_seg==0) {
        // セグメントが無い場合は全てのk_valueに対して0を返す
        for(auto kv: k_values) {
            (void)kv; // unused
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
        if(n_seg>0) retention = (double)count_over/(double)n_seg;
        ret.push_back(retention);
    }
    return ret;
}

static pair<double,double> compute_variance_std(
    const vector<vector<double>> &probability
) {
    vector<double> all_p;
    for(auto &pv: probability) {
        for(auto val: pv){
            all_p.push_back(val);
        }
    }

    if(all_p.empty()){
        // 全くセグメントが無い場合など
        return {0.0,0.0};
    }

    double mean=0.0;
    for(auto val: all_p) mean+=val;
    mean/= (double)all_p.size();
    double var=0.0;
    for(auto val: all_p){
        double diff=(val-mean);
        var+= diff*diff;
    }
    var/= (double)all_p.size();
    double stddev = sqrt(var);
    return {var,stddev};
}

int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // パラメータをコード内で指定
    m = 40;
    a =30;       // aを増やしてもSegfaultしないように修正済み
    num_solve = 100;
    d = 2;
    k_values = {0.1, 0.2, 0.3};

    starts.resize(a);
    goals.resize(a);
    for(int i=0;i<a;i++){
        starts[i] = { (int)(rng()%m), (int)(rng()%m) };
        goals[i] = { (int)(rng()%m), (int)(rng()%m) };
    }

    vector<vector<Path>> solutions_with = run_solutions(true);
    vector<vector<Path>> solutions_without = run_solutions(false);

    // セグメント抽出
    AgentSolutionSegments ass_with = extract_segments(solutions_with);
    AgentSolutionSegments ass_without = extract_segments(solutions_without);

    // 通過確率計算
    vector<vector<double>> probability_with = compute_pass_probabilities(solutions_with, ass_with);
    vector<vector<double>> probability_without = compute_pass_probabilities(solutions_without, ass_without);

    vector<Segment> all_segments_with = to_vector(ass_with.all_segments_set);
    vector<Segment> all_segments_without = to_vector(ass_without.all_segments_set);

    vector<double> data_retention_with = compute_data_retention(probability_with, all_segments_with);
    vector<double> data_retention_without = compute_data_retention(probability_without, all_segments_without);

    auto [var_with, std_with] = compute_variance_std(probability_with);
    auto [var_without, std_without] = compute_variance_std(probability_without);

    cout << "Collision avoidance count (with): " << collision_avoidance_count_with << "\n";
    cout << "Data retention (with):";
    for(auto v:data_retention_with) cout<<" "<<v;
    cout<<"\n";
    cout << "Data retention (without):";
    for(auto v:data_retention_without) cout<<" "<<v;
    cout<<"\n";
    cout << "Variance (with): " << var_with << ", StdDev (with): " << std_with << "\n";
    cout << "Variance (without): " << var_without << ", StdDev (without): " << std_without << "\n";

    return 0;
}
