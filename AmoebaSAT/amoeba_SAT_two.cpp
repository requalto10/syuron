#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <tuple>
#include <chrono>
#include <cassert>

/**
 * (0,1)の実数乱数を生成する
*/
class Random {
  public:
    Random() {
      std::random_device seed_gen;  // 物理乱数 
      engine_ = std::mt19937(seed_gen());
      dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    }

    Random(int seed) {
      engine_ = std::mt19937(seed);
      dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    }

    double next() {
      return dist_(engine_);
    }

  private:
    std::mt19937 engine_;  // TODO: ここですでに初期化されているのかもしれないのでコンストラクタで改めて作る必要ないかも
    std::uniform_real_distribution<> dist_;
};

// mainで1回だけ初期化し、ファイル内全体で使用するためグローバルに宣言する
Random rand_gen;

class FlactuationGenerator {
  public:
    FlactuationGenerator() {
      value_ = rand_gen.next();
    }
    
    double next() {
      // Logistic Map
      value_ = 4 * value_ * (1 - value_);
      return value_;
    }
  
  private:
    double value_;
};

class Flactuation {
  public:
    Flactuation() {}

    int apply(int bounce) {
      if (bounce == 1) {
        return 0;
      } else {
        return 1 - exploration_param - z_.next() > 0;
        return 1 - exploration_param - z_.next() > 0;
      }
    }

  private:
    const double exploration_param = 0.09;  // Amoeba SAT資料 p27
    FlactuationGenerator z_;
};

struct index {
  int j;
  int b;
};

class AmoebaLeg {
  public:
    AmoebaLeg(int state) : state_(state) {}

    int state() const {
      return state_;
    }

    void change(int demand, int prev) {
      if (demand == 0 && state_ > -1) {
        state_--;
      } else if (demand == 1 && state_ < 1) {
        state_++;
      }
    }

  private:
    int state_;
};

class Variable {
  public:
    Variable(int j) : j_(j), value_(0) {
      prev_[0] = 0;
      prev_[1] = 0;
    }

    ~Variable() {}

    int value() const {
      return value_;
    }

    AmoebaLeg get_leg(int b) const {
      return x_[b];
    }

    int j() const {
      return j_;
    }

    int get_current(int b) const {
      return x_[b].state();
    }

    int get_previous(int b) const {
      return prev_[b];
    }

    void update(int change_demands0, int change_demands1) {
      prev_[0] = x_[0].state();
      prev_[1] = x_[1].state();
      x_[0].change(change_demands0, prev_[0]);
      x_[1].change(change_demands1, prev_[1]);

      if (x_[0].state() == 1 && x_[1].state() <= 0) {
        value_ = 0;
      } else if (x_[0].state() <= 0 && x_[1].state() == 1) {
        value_ = 1;
      } else {
        // value_はそのまま
      }
    }

  private:
    int j_;
    AmoebaLeg x_[2]{0, 0};
    int prev_[2];
    int value_;
};

/**
 * index(j,b)ごとに集めたバウンスバックルール
*/
class Rule {
  struct inter {
    index idx[2];
  };

  public:
    Rule() {}

    void set_index(index idx) {
      idx_ = idx;
      intra_.j = idx.j;
      intra_.b = idx.b ^ 1;  // 1とXORを取る(最下位ビットを反転)
    }

    int apply(const std::vector<Variable*> &vars) const;

    void add_inter(index idx1, index idx2) {
      inters_.push_back({idx1, idx2});
    }

    std::string str() const {
      std::stringstream ss;
      for (auto& inter : inters_) {
        ss << "((" << inter.idx[0].j << "," << inter.idx[0].b << "),"
          << "(" << inter.idx[1].j << "," << inter.idx[1].b << ")), ";
      }
      return ss.str();
    }

  private:
    index idx_;
    index intra_;
    std::vector<inter> inters_;
};

int Rule::apply(const std::vector<Variable*> &vars) const {
  // varsは添字jの順に並んでいる前提
  
  // Check INTRA
  if (vars[intra_.j]->get_leg(intra_.b).state() == 1) {
    return 1;
  }

  // Check INTER
  for (auto inter : inters_) {
    if (
      vars[inter.idx[0].j]->get_leg(inter.idx[0].b).state() == 1
      && vars[inter.idx[1].j]->get_leg(inter.idx[1].b).state() == 1
    ) {
      return 1;
    }
  }

  return 0;
}

class BouncebackSignal {
  public:
    BouncebackSignal() {}

    void set_index(index idx) {
      idx_ = idx;
      rule_.set_index(idx);
    }

    int current() const {
      return current_;
    }

    int previous() const {
      return previous_;
    }

    Rule& rule() {
      return rule_;
    }

    void update(const std::vector<Variable*> &vars) {
      previous_ = current_;
      current_ = rule_.apply(vars);
    }

  private:
    int current_ = 0;
    int previous_ = 0;
    index idx_;
    Rule rule_;
};

class Bounceback {
  public:
    Bounceback(int j) : j_(j) {
      signal_[0].set_index({j, 0});
      signal_[1].set_index({j, 1});
    }

    int j() const {
      return j_;
    }

    int get_change_demand(int b) const {
      return change_demand_[b];
    }

    void update(const std::vector<Variable*> &vars);

    BouncebackSignal& get_signal(int b) {
      return signal_[b];
    }

  private:
    int j_;
    BouncebackSignal signal_[2];
    Flactuation flactuation_[2];
    int change_demand_[2];
};

void Bounceback::update(const std::vector<Variable*> &vars) {
  signal_[0].update(vars);
  signal_[1].update(vars);

  // b=0,1の両方の暫定バウンスバック信号が1の場合、50/50の確率でどちらかを0にする
  int fixed_bounce_signal0;
  int fixed_bounce_signal1;
  if (signal_[0].current() == 1 && signal_[1].current() == 1) {
    // [0,1]の実数乱数を使う
    if (rand_gen.next() < 0.5) {
      fixed_bounce_signal0 = 0;
      fixed_bounce_signal1 = 1;
    } else {
      fixed_bounce_signal0 = 1;
      fixed_bounce_signal1 = 0;
    }
  } else {
    fixed_bounce_signal0 = signal_[0].current();
    fixed_bounce_signal1 = signal_[1].current();
  }

  change_demand_[0] = flactuation_[0].apply(fixed_bounce_signal0);
  change_demand_[1] = flactuation_[1].apply(fixed_bounce_signal1);

  // デバッグ用に出力
  // std::cout << fixed_bounce_signal0 << "," << fixed_bounce_signal1 << " ";
}
  
std::tuple<int, std::vector<std::vector<int>>> read_cnf_file(std::string file_name) {
    std::ifstream file(file_name);
    std::vector<std::vector<int>> cnf;
    int num_vars;
    std::string line;

    while (getline(file, line)) {
        // if (line.empty() || line[0] == 'c' || line[0] == 'p') {
        if (line.empty() || line[0] == 'c') {
            continue; // コメント行を飛ばす
        }

        if (line[0] == '%') {
          break; // この行で終了
        }

        std::vector<int> clause;
        int var;

        std::istringstream iss(line);

        if (line[0] == 'p') {
          // 問題の情報行
          std::string tmp;
          iss >> tmp >> tmp >> num_vars;
          continue;
        }


        while (iss >> var) {
            if (var == 0) {
                break; // 各節項は0で終了する
            }
            clause.push_back(var);
        }

        cnf.push_back(clause);
    }

    return {num_vars, cnf};
}

/**
 * バウンスバックルールのINTERを作り、`rules`に格納する
*/
void make_inter(const std::vector<std::vector<int>> &cnf, std::vector<Bounceback*> &bouncebacks) {
  for (auto& clause : cnf) {
    // 各リテラルを添字jと真偽b(0or1)に分ける
    // CNFでは添字は{1...N}だが、配列のインデックスで使うため{0...N-1}とする
    // バウンスバックでは、節が0になることを禁止するので、正リテラルなら0、負リテラルなら1になることを禁止する
    index a = {abs(clause[0]) - 1, clause[0] > 0 ? 0: 1};
    index b = {abs(clause[1]) - 1, clause[1] > 0 ? 0: 1};
    index c = {abs(clause[2]) - 1, clause[2] > 0 ? 0: 1};

    bouncebacks[a.j]->get_signal(a.b).rule().add_inter(b, c);
    bouncebacks[b.j]->get_signal(b.b).rule().add_inter(c, a);
    bouncebacks[c.j]->get_signal(c.b).rule().add_inter(a, b);
  }
}

bool check_stability_v1(const std::vector<Variable*> &vars, const std::vector<Bounceback*> &bouncebacks) {
  assert(vars.size() == bouncebacks.size());

  for (std::size_t i = 0; i < vars.size(); i++) {
    bool is_stable_0 = vars[i]->get_current(0) + vars[i]->get_previous(0) >= 1
                    && bouncebacks[i]->get_signal(0).current() + bouncebacks[i]->get_signal(0).previous() == 0
                    && vars[i]->get_current(1) + vars[i]->get_previous(1) <= 0
                    && bouncebacks[i]->get_signal(1).current() + bouncebacks[i]->get_signal(1).previous() >= 1;
    
    bool is_stable_1 = vars[i]->get_current(1) + vars[i]->get_previous(1) >= 1
                    && bouncebacks[i]->get_signal(1).current() + bouncebacks[i]->get_signal(1).previous() == 0
                    && vars[i]->get_current(0) + vars[i]->get_previous(0) <= 0
                    && bouncebacks[i]->get_signal(0).current() + bouncebacks[i]->get_signal(0).previous() >= 1;
    
    if (is_stable_0 || is_stable_1) {
      // OK
    } else {
      return false;
    }
  }
  
  return true;
}

bool check_stability_v2(const std::vector<Variable*> &vars, const std::vector<Bounceback*> &bouncebacks) {
  assert(vars.size() == bouncebacks.size());

  for (std::size_t i = 0; i < vars.size(); i++) {
    
    bool is_stable_0_a = vars[i]->get_current(0) - bouncebacks[i]->get_signal(0).current() >= 0
                        && vars[i]->get_previous(0) - bouncebacks[i]->get_signal(0).previous() > 0
                        && vars[i]->get_current(1) - bouncebacks[i]->get_signal(1).current() <=-1;

    bool is_stable_0_b = vars[i]->get_current(0) - bouncebacks[i]->get_signal(0).current() > 0
                        && vars[i]->get_previous(0) - bouncebacks[i]->get_signal(0).previous() >= 0
                        && vars[i]->get_current(1) - bouncebacks[i]->get_signal(1).current() <=-1;

    bool is_stable_1_a = vars[i]->get_current(1) - bouncebacks[i]->get_signal(1).current() >= 0
                        && vars[i]->get_previous(1) - bouncebacks[i]->get_signal(1).previous() > 0
                        && vars[i]->get_current(0) - bouncebacks[i]->get_signal(0).current() <=-1;

    bool is_stable_1_b = vars[i]->get_current(1) - bouncebacks[i]->get_signal(1).current() > 0
                        && vars[i]->get_previous(1) - bouncebacks[i]->get_signal(1).previous() >= 0
                        && vars[i]->get_current(0) - bouncebacks[i]->get_signal(0).current() <=-1;

    if (is_stable_0_a || is_stable_0_b ||  is_stable_1_a || is_stable_1_b) {
      // OK
    } else {
      return false;
    }
  }
  
  return true;
}

bool test_sat_all_clause(const std::vector<std::vector<int>> &cnf, const std::vector<Variable*> &vars) {
  for (const auto& clause : cnf) {
    int i = abs(clause[0]) - 1;
    int j = abs(clause[1]) - 1;
    int k = abs(clause[2]) - 1;
    int literal_i = clause[0] > 0 ? 1 : 0;
    int literal_j = clause[1] > 0 ? 1 : 0;
    int literal_k = clause[2] > 0 ? 1 : 0;

    if (vars[i]->value() == literal_i || vars[j]->value() == literal_j || vars[k]->value() == literal_k) {
      // OK
    } else {
      // NG
      return false;
    }
  }
  return true;
}

void update_bounceback(std::vector<Bounceback*> &bouncebacks, const std::vector<Variable*> &vars) {
  for (auto& bounceback : bouncebacks) {
    bounceback->update(vars);
  }
}

void update_variables(std::vector<Variable*> &vars, const std::vector<Bounceback*> &bouncebacks) {
  assert(vars.size() == bouncebacks.size());
  
  for (std::size_t i = 0; i < vars.size(); i++) {
    vars[i]->update(bouncebacks[i]->get_change_demand(0), bouncebacks[i]->get_change_demand(1));
  }
}

void print_inter(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "# Inter" << std::endl;
  for (auto& bounceback : bouncebacks) {
    std::cout << "(" << bounceback->j() << ",0): "
      << bounceback->get_signal(0).rule().str() << std::endl;

    std::cout << "(" << bounceback->j() << ",1): "
      << bounceback->get_signal(1).rule().str() << std::endl;
  }
  std::cout << std::endl;
}

void print_bouncebacks(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "Raw Bounceback Signals: ";
  for (auto& bounceback : bouncebacks) {
      std::cout << bounceback->get_signal(0).current();
      std::cout << ",";
      std::cout << bounceback->get_signal(1).current();
      std::cout << " ";
  }
  std::cout << std::endl;
}

void print_change_demands(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "Change Demands: ";
  for (auto& bounceback : bouncebacks) {
      std::cout << bounceback->get_change_demand(0);
      std::cout << ",";
      std::cout << bounceback->get_change_demand(1);
      std::cout << " ";
  }
  std::cout << std::endl;
}

void print_variables_leg(const std::vector<Variable*> &vars) {
  std::cout << "Amoeba Legs: ";
  for (auto& var : vars) {
    std::cout << var->get_current(0) << "," << var->get_current(1) << " ";
  }
  std::cout << std::endl;
}

void print_variables(const std::vector<Variable*> &vars) {
  std::cout << "Solution: ";
  for (auto& var : vars) {
    std::cout << var->value() << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char* argv[]) {
  const int max_iteration = 10000000;

  if (argc != 3) {
    std::cout << "Usage: ./program <file_name> <seed>" << std::endl;
    return 1;
  }
  
  std::string file_name(argv[1]);
  // global変数rand_genの初期化
  int seed = std::stoi(argv[2]);
  rand_gen = Random(seed);

  // タイム測定開始
  auto start = std::chrono::high_resolution_clock::now();

  auto [num_vars, cnf] = read_cnf_file(file_name);

  std::vector<Variable*> vars(num_vars);
  std::vector<Bounceback*> bouncebacks(num_vars);

  for (int i = 0; i < num_vars; i++) {
    index idx[2] = {{i, 0}, {i, 1}};

    vars[i] = new Variable(i);
    bouncebacks[i] = new Bounceback(i);
  }

  make_inter(cnf, bouncebacks);
  // print_inter(bouncebacks);

  // std::cout << "Searching";
  // std::cout.flush();
  bool is_sat = false;
  int i;
  for (i = 0; i < max_iteration; i++) {

    update_bounceback(bouncebacks, vars);
    
    // 資料AmoebaSAT_instructionとATOswitchv1.0より、更新順番は下記の通り
    // X(t) -> L(t) -> S(t+1) -> X(t+1) ->L(t+1)
    // 解の判定条件はX(t), X(t+1), L(t), L(t+1)の関数になるので、
    // バウンスバック信号を更新したタイミングで解の判定を実行する
    if (check_stability_v1(vars, bouncebacks)) {
    // if (check_stability_v2(vars, bouncebacks)) {
    // if (test_sat_all_clause(cnf, vars)) {
      is_sat = true;
      break;
    }
    
    update_variables(vars, bouncebacks);

    // デバッグ用のためコメントアウト
    // print_bouncebacks(bouncebacks);
    // print_change_demands(bouncebacks);
    // print_variables_leg(vars);


    // 進捗状況表示
    // if (i % 100000 == 0) {
    //   std::cout << ".";
    //   std::cout.flush();
    // }
  }
  // タイム測定終了
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);


  // 解の出力
  if (is_sat) {
    // print_variables(vars);
    assert(test_sat_all_clause(cnf, vars));
  } else {
    // std::cout << "Solution not reached." << std::endl;
  }

  // 結果の出力、下記をスペース区切りで出力する
  // seed SAT/UNSAT #Iteration time(us)
  std::string result = is_sat ? "SAT" : "UNSAT";
  std::cout << seed << " " << result << " " << i << " " << duration.count() << std::endl;

  // TODO: メモリリークしていないか調べる、オブジェクト内の配列はどうなのか

  for (int i = 0; i < num_vars; i++) {
    delete vars[i];
    delete bouncebacks[i];
  }

  return 0;
}
