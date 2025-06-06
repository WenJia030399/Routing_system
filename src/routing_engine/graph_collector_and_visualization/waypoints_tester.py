import json
import random
import subprocess

def generate_task_json(filename="/home/wenjia/isaac_routing2/isaac_routing/src/routing_engine/graph_collector_and_visualization/task_data_empty.json"):
    # 任務池
    tasks = []
    for i in range(random.randint(1,7)):
        tasks.append("000_"+str(random.randint(1,200)).zfill(3))
    
    # 每個任務需求量為 1（也可改成隨機）
    demand = [1] * len(tasks)

    # 準備 JSON 格式
    data = {
        "task_sequence": tasks,
        "demand": demand
    }

    # 寫入 JSON 檔
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)
    
    print(f"已隨機生成 JSON 檔案：{filename}")

def launch_ros2():
    try:
        # 確保有 ROS 環境（換成你的 setup 路徑）
        subprocess.run(
            "source /opt/ros/humble/setup.bash && ros2 launch routing_agent RunServer.launch.py",
            shell=True,
            executable="/bin/bash",
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"執行 ROS launch 失敗：{e}")

if __name__ == "__main__":
    generate_task_json()
    launch_ros2()
