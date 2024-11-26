# ATO_viewer

## 概要

ATOの結果(探索の時系列)からmovieを作成する python スクリプトである。


## ファイル・ディレクトリ構成

このディレクトリは以下の構成を持つ：

```bash
ATO_viwer
├── ATO_viewer.py              スクリプト本体
├── README_ja.md               このファイル
├── data                       テスト用データファイル
├── grid_eval_A_layout.json    "A" layout 用 position マップ
├── grid_eval_B_layout.json    "B" layout 用 position マップ
├── make_map.py                上のファイルを作るのに用いたスクリプト
├── movies                     出力動画用ディレクトリ
└── run.sh                     動作確認用シェルスクリプト

```

## 前提

本プログラムでは、python3の他に、以下のアプリケーションおよびpythonライブラリがインストールされていることを前提とする:

- アプリケーション
  - ffmpeg
- pythonライブラリ
  - tqdm
  - matplotlib
  - numpy
 
 また、pythonのバージョンは 3.9を利用して確認した。
 
## 前提となるアプリケーションおよびpythonライブラリのインストール方法

### アプリケーション

MacOSで利用している場合、homebrew で簡単にインストール可能である。

```bash
# brew install ffmpeg
```

homebrew 本体のインストール方法については、https://brew.sh を参照のこと。


Ubuntuで利用する場合は、aptパッケージマネージャでインストールが可能である。(ただしsudoerか スーパユーザー権限が必要)

```bash
# sudo apt install ffmpeg
```

### pythonライブラリ

```bash
# pip3 install matplotlib tqdm numpy
```

なお、matplotlibはあらかじめインストールされている場合が多いが、バージョンが低いと
3D機能が使えない。その場合は

```bash
# pip3 install -U matplotlib 
```

とすれば最新版にアップグレード可能である。また、numpyのバージョンはpytorchなどをインストールしている場合は
衝突しやすいので注意すること。

## 実行方法

実行方法は以下の通りである:

```bash
# python3 ATO_viwer.py [結果ファイルのパス] [オプション（省略可）]
```
- `[結果ファイルのパス]`: ATOで作成された結果ファイルのパスで省略不可である。

## オプション
- `--plate-data`: プレートのデータファイルへのパスを指定。省略するとプレートの描画はされない
- `--plate-size`: プレートのサイズ。省略した場合、デフォルト値として$5$が設定される
- `--output movieファイルのパス`: 出力するmovieファイルのパスである。省略すると結果ファイルのパスの拡張子を`mp4`に変えたファイルが
  出力される。拡張子は `mp4`とすること。それ以外の拡張子をつけた場合の動作は保証しない。
- `--step イテレーション番号`: 特定のイテレーション番号(t)だけをMatplotlibを利用して表示。マウスで回転させることなどができる。`--output`オプションは無視される。
- `--view 仰角 方位角`: 3D視点の角度を設定する。ラジアンではなく度数で設定すること。数値は必ず２つ、スペースを入れて設定すること。省略するとそれぞれ30度に設定される。
- `--position-map マップファイルのパス`: セル番号とセルの位置座標の対応を定義したjsonファイルのパスを設定する。省略すると6x6グリッドが設定される
- `--ranges x_min x_max y_min y_max`: x,y座標の範囲を設定。必ず4つ数字をスペースを開けて設定すること。省略すると、マップファイルから自動的に範囲が設定させる
- `--cars`: 特定の台車だけを表示。複数設定する場合は、スペースを開けて数値を並べること。省略した場合は全台車が描画される
- `--zscale`: z方向のスケールを設定。デフォルト値は1
- `--cell-base`: 与えられたデータが頂点ベースであることをツールに知らせるフラグ。省略すると辺ベースになる

テスト用のシェルスクリプトを用意している。

```bash
 ./run.sh
```

このスクリプトは data ディレクトリにある結果データをmovieにする。

## 引数の例

以下の様に与える：

```bssh
# python3 ATO_viewer.py data/AmoebaEnergy/Projects/ATO_Viewer_ver4/data/evolution_U_grid_36_4D_v4_01_1.dat  --output movies/evolution_U_grid_36_4D_v4_01_1.dat movies/.mp4
```

また、例えば100ステップのスナップショットを様々な角度で観察したい場合は、

```bssh
# python3 ATO_viewer.py data/AmoebaEnergy/Projects/ATO_Viewer_ver4/data/evolution_U_grid_36_4D_v4_01_1.dat  --output movies/evolution_U_grid_36_4D_v4_01_1.dat --step 100
```

movie の視点を変えたい時は、`--view`オプションを使う:

```bssh
# python3 ATO_viewer.py data/AmoebaEnergy/Projects/ATO_Viewer_ver4/data/evolution_U_grid_36_4D_v4_01_1.dat  --output movies/evolution_U_grid_36_4D_v4_01_1.dat --view 30 50
```
プレートデータがあるならば、Uデータと同時に指定することでプレートも表示できる：

```bssh
# python3 ATO_viewer.py data/AmoebaEnergy/Projects/ATO_Viewer_ver4/data/evolution_U_grid_36_4D_v4_01_1.dat  --output movies/evolution_U_grid_36_4D_v4_01_1.dat --plate-data data/AmoebaEnergy/Projects/ATO_Viewer_ver4/data/evolution_C_grid_36_4D_v4_01_1.dat
```

## 結果データについて

結果データは以下の形式のテキストファイルでなければならない:

```python
---
ヘッダ情報
---
[[台車1], [台車2], ...., [台車N]]
```

一行にあるステップの台車N台分の情報をpythonと同じ形式の配列の形で表現する。`[台車n]`は配列の配列であり、
内部の配列は一つのセル」を表現する。各台車において`U=1`となっているU変数で、頂点ベースの場合、

`[[時間ステップp, セル番号, 方向, オプション(x or z)], [...], [...], ....]`

辺ベースの場合、

`[[時間ステップp, 支店セル番号, 終点セル番号, 足の高さ], [...], [...], ....]`

でなければならない。

## map ファイルについて

map ファイルはjsonでなければならない。読み込まれた時に、keyが番号で、valueが[x,y]の配列に
なるようなjsonファイルとする。例えば以下の様に書く：

```json
{
	"1": [0, 0],
	"2": [1, 0],
	"3": [3, 0]
}
```

以上







