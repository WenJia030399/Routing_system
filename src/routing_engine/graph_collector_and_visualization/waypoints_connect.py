import json

# 讀取原始檔案
input_file = input("請輸入原始檔案名稱: ") 
output_file = "Done_"+input_file

# 載入資料
with open(input_file, 'r') as f:
    data = json.load(f)

# 取得所有節點 ID，並依照編號排序
node_ids = sorted(data.keys(), key=lambda x: int(x.split('_')[1]))

# 根據排序依序建立雙向連線
for i in range(len(node_ids)):
    current_node = node_ids[i]
    edges = set(data[current_node]["edges"])

    # 加入前一個節點
    if i > 0:
        prev_node = node_ids[i - 1]
        edges.add(prev_node)
        data[prev_node]["edges"] = list(set(data[prev_node]["edges"]) | {current_node})

    # 加入下一個節點
    if i < len(node_ids) - 1:
        next_node = node_ids[i + 1]
        edges.add(next_node)
        data[next_node]["edges"] = list(set(data[next_node]["edges"]) | {current_node})

    # 更新 edges
    data[current_node]["edges"] = sorted(edges)

# 寫入新檔案
with open(output_file, 'w') as f:
    json.dump(data, f, indent=2)

print(f" 處理完成！已儲存至 {output_file}")
