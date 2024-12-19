import sys

# 処理対象のファイル名を引数から取得（なければ標準入力から）
filename = sys.argv[1] if len(sys.argv) > 1 else None

# ファイルオブジェクトを確保
if filename:
    f = open(filename, 'r', encoding='utf-8')
else:
    f = sys.stdin

with f:
    for line in f:
        line = line.strip()
        # データ残留率(%):が含まれる行を探索
        # 例: "   データ残留率(%): 49.8918 14.6514 5.78125"
        if "データ残留率(%):" in line:
            # 行を空白で分割
            parts = line.split()
            # partsは ["データ残留率(%):", "49.8918", "14.6514", "5.78125"] のような構造になる想定
            # 最初の要素は"データ残留率(%):"なので、それ以降の値を1つずつ改行表示
            for val in parts[1:]:
                print(val)
