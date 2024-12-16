import re

# ファイルパスを指定
file_path = "/Users/keiten/Documents/Master/MAPF/GenPaths/C++/Seg_with_Probability/100x100_d3/1213-0414.txt"

# Data residual rateを格納するリスト
data_residual_rates = []

# ファイルを読み込んで走査
with open(file_path, "r") as file:
    for line in file:
        match = re.search(r"Data residual rate: ([\d.]+)%", line)
        if match:
            # パーセント記号を除いて数値のみを抽出
            data_residual_rates.append(match.group(1))

# 標準出力に改行しながら出力
for rate in data_residual_rates:
    print(rate)
