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

    int next() {
      double random = z_.next();
      if (random < 1 - exploration_param) {
        return 0;
      } else {
        // z_がロジスティックマップの場合、確率分布は均等にならない
        // 1か-1をシステムの乱数を使って決める
        if (rand_gen.next() < 0.5) return -1;
        else return 1;
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
    AmoebaLeg() {
      // state_をランダムに-1,0,1で初期化する
      double rand = rand_gen.next() * 3;  // (0,3)の実数乱数
      if (rand < 1) {
        state_ = -1;
      } else if (rand < 2) {
        state_ = 0;
      } else {
        state_ = 1;
      }
    }

    int state() const {
      return state_;
    }

    void change(int demand) {
      if (demand == -1 && state_ > -1) {
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
    Variable(int j) : j_(j) {
      prev_ = 0;
    }

    ~Variable() {}

    int value() const {
      return value_;
    }

    AmoebaLeg get_leg() const {
      return x_;
    }

    int j() const {
      return j_;
    }

    int get_current() const {
      return x_.state();
    }

    int get_previous() const {
      return prev_;
    }

    void update(int change_demands) {
      prev_ = x_.state();
      x_.change(change_demands);

      // 足の状態から変数値への変換は2つの方法がある
      // AとBの方法で、ステップ数に差はほとんどない
      // ただし、Bの方法はstab_check_v2のときのみ使える
      // if (x_.state() == -1) {  // A
      if (x_.state() + prev_ < -1) {  // B
        value_ = 0;
      // } else if (x_.state() == 1) {  // A
      } else if (x_.state() + prev_ > 1) {  // B
        value_ = 1;
      } else {
        // value_はそのまま
      }
    }

  private:
    int j_;
    AmoebaLeg x_;
    int prev_;
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
    std::vector<inter> inters_;
};

int Rule::apply(const std::vector<Variable*> &vars) const {
  // varsは添字jの順に並んでいる前提
  
  // Check INTER
  for (auto inter : inters_) {
    if (
      vars[inter.idx[0].j]->get_leg().state() == inter.idx[0].b
      && vars[inter.idx[1].j]->get_leg().state() == inter.idx[1].b
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
      l_signal_.set_index({j, 0});
      f_signal_.set_index({j, 1});
    }

    int j() const {
      return j_;
    }

    int get_change_demand() const {
      return change_demand_;
    }

    void update(const std::vector<Variable*> &vars);

    BouncebackSignal& get_signal(int b) {
      assert(b == 1 || b == -1);
      if (b == 1) {
        return f_signal_;
      } else {
        return l_signal_;
      }
    }

  private:
    int j_;
    BouncebackSignal l_signal_;
    BouncebackSignal f_signal_;
    Flactuation flactuation_;
    int change_demand_;
};

void Bounceback::update(const std::vector<Variable*> &vars) {
  l_signal_.update(vars);
  f_signal_.update(vars);

  // b=0,1の両方の暫定バウンスバック信号が1の場合、50/50の確率でどちらかを0にする
  int fixed_l_signal;
  int fixed_f_signal;
  if (l_signal_.current() == 1 && f_signal_.current() == 1) {
    // [0,1]の実数乱数を使う
    if (rand_gen.next() < 0.5) {
      fixed_l_signal = 0;
      fixed_f_signal = 1;
    } else {
      fixed_l_signal = 1;
      fixed_f_signal = 0;
    }
  } else {
    fixed_l_signal = l_signal_.current();
    fixed_f_signal = f_signal_.current();
  }

  if (fixed_l_signal == 0 && fixed_f_signal == 1) {
    change_demand_ = 1;
  } else if (fixed_l_signal == 1 && fixed_f_signal == 0) {
    change_demand_ = -1;
  } else {
    change_demand_ = flactuation_.next();
  }
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
    // バウンスバックでは、節が0になることを禁止するので、正リテラルなら0、負リテラルなら1になるときにルール
    index a = {abs(clause[0]) - 1, clause[0] > 0 ? -1: 1};
    index b = {abs(clause[1]) - 1, clause[1] > 0 ? -1: 1};
    index c = {abs(clause[2]) - 1, clause[2] > 0 ? -1: 1};

    bouncebacks[a.j]->get_signal(-1 * a.b).rule().add_inter(b, c);
    bouncebacks[b.j]->get_signal(-1 * b.b).rule().add_inter(c, a);
    bouncebacks[c.j]->get_signal(-1 * c.b).rule().add_inter(a, b);
  }
}

bool check_stability_v2(const std::vector<Variable*> &vars, const std::vector<Bounceback*> &bouncebacks) {
  assert(vars.size() == bouncebacks.size());

  for (std::size_t i = 0; i < vars.size(); i++) {
    int x_bar = vars[i]->get_current() + vars[i]->get_previous();
    int l_bar = bouncebacks[i]->get_signal(-1).current() + bouncebacks[i]->get_signal(-1).previous();
    int f_bar = bouncebacks[i]->get_signal(1).current() + bouncebacks[i]->get_signal(1).previous();

    bool is_stable_1 = x_bar > 1 && f_bar - l_bar > 1;
    bool is_stable_0 = x_bar < -1 && f_bar - l_bar < -1;
    bool is_stable_free = f_bar + l_bar == 0;
    
    if (is_stable_1 || is_stable_0 || is_stable_free) {
      // OK
    } else {
      return false;
    }
  }
  
  return true;
}

bool check_stability_v1(const std::vector<Variable*> &vars, const std::vector<Bounceback*> &bouncebacks) {
  assert(vars.size() == bouncebacks.size());

  for (std::size_t i = 0; i < vars.size(); i++) {
    int x_bar = vars[i]->get_current() + vars[i]->get_previous();
    int l_bar = bouncebacks[i]->get_signal(-1).current() + bouncebacks[i]->get_signal(-1).previous();
    int f_bar = bouncebacks[i]->get_signal(1).current() + bouncebacks[i]->get_signal(1).previous();

    bool is_stable_1_forced = x_bar >= 1 && l_bar == 0 && f_bar >= 1;
    bool is_stable_0_forced = x_bar <= -1 && l_bar >= 1 && f_bar == 0;
    bool is_stable_1_free = x_bar >= 1 && l_bar + f_bar == 0;
    bool is_stable_0_free = x_bar <= -1 && l_bar + f_bar == 0;
    // bool is_stable_free = x_bar + l_bar + f_bar == 0;  // 怪しい、x_barは-2から2だが、l_barとf_barは0から2なので

    // if (is_stable_1_forced || is_stable_0_forced || is_stable_1_free || is_stable_0_free || is_stable_free) {
    if (is_stable_1_forced || is_stable_0_forced || is_stable_1_free || is_stable_0_free) {
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
    vars[i]->update(bouncebacks[i]->get_change_demand());
  }
}

void print_inter(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "# Inter" << std::endl;
  for (auto& bounceback : bouncebacks) {
    std::cout << bounceback->j() << "-l: "
      << bounceback->get_signal(-1).rule().str() << std::endl;

    std::cout << bounceback->j() << "-f: "
      << bounceback->get_signal(1).rule().str() << std::endl;
  }
  std::cout << std::endl;
}

void print_bouncebacks(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "Raw Bounceback Signals: ";
  for (auto& bounceback : bouncebacks) {
      std::cout << bounceback->get_signal(-1).current();
      std::cout << ",";
      std::cout << bounceback->get_signal(1).current();
      std::cout << " ";
  }
  std::cout << std::endl;
}

void print_change_demands(const std::vector<Bounceback*> &bouncebacks) {
  std::cout << "Change Demands: ";
  for (auto& bounceback : bouncebacks) {
      std::cout << bounceback->get_change_demand();
      std::cout << " ";
  }
  std::cout << std::endl;
}

void print_variables_leg(const std::vector<Variable*> &vars) {
  std::cout << "Amoeba Legs: ";
  for (auto& var : vars) {
    std::cout << var->get_current() << " ";
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
    // if (check_stability_v1(vars, bouncebacks)) {
    if (check_stability_v2(vars, bouncebacks)) {
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
