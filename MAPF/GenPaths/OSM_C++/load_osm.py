import osmnx as ox
import networkx as nx
import json

# さいたま市の道路ネットワークを取得
place_name = "Saitama, Japan"
G = ox.graph_from_place(place_name, network_type='drive')

# 必要なデータを抽出
graph_data = {
    "nodes": [],
    "edges": []
}

# ノードの座標
for node, data in G.nodes(data=True):
    graph_data["nodes"].append({
        "id": node,
        "x": data['x'],
        "y": data['y']
    })

# エッジの重み（距離）
for u, v, data in G.edges(data=True):
    graph_data["edges"].append({
        "source": u,
        "target": v,
        "weight": data.get('length', 1)  # エッジの長さを重みとする
    })

# JSON形式で保存
with open("saitama_graph.json", "w") as f:
    json.dump(graph_data, f)
