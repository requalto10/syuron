# AmoebaSATのC++実装

## ビルド

```
g++ amoeba_SAT_one.cpp -o amoeba_SAT_one
g++ amoeba_SAT_two.cpp -o amoeba_SAT_two
```

## 実行

### 単体で実行

```
./amoeba_SAT_[one/two] [cnf file] [seed]
```

例
```
./amoeba_SAT_one uf50-0100.cnf 1
```

### 実行結果

```
[seed] [SAT/UNSAT] [#iteration] [かかった実時間(マイクロ秒)]
```

例
```
1 SAT 220 4976
```

### 複数回繰り返して実行

繰り返し回数(count)を指定して実行する。
各実行のシードは0,1,2,...,count-1となる。

```
./exec.sh [実行ファイル] [cnf file] [count]
```

例
```
./exec.sh amoeba_SAT_two uf50-0100.cnf 50
```