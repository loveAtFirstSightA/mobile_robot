import matplotlib.pyplot as plt
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

# 创建图形
plt.figure(figsize=(10, 12))
plt.title('X-axis forward facing upward', fontsize=16)

# 第一象限数据
q1_path = [[1.0, 1.0], [1.0, 2.0], [1.0, 3.0], [1.0, 4.0], [1.0, 5.0], [1.0, 6.0], [1.0, 7.0], [1.0, 8.0], [1.0, 9.0], [1.0, 10.0], 
           [1.0, 11.0], [1.0, 12.0], [1.0, 13.0], [1.0, 14.0], [1.0, 15.0], [1.0, 16.0], [1.0, 17.0], [1.0, 18.0], [1.0, 19.0], [1.0, 20.0],
           [2.0, 1.0], [2.0, 8.0], [2.0, 10.0], [2.0, 17.0], [2.0, 19.0],
           [3.0, 1.0], [3.0, 2.0], [3.0, 7.0], [3.0, 8.0], [3.0, 10.0], [3.0, 11.0], [3.0, 16.0], [3.0, 17.0], [3.0, 18.0], [3.0, 19.0], [3.0, 20.0],
           [4.0, 1.0], [4.0, 8.0], [4.0, 10.0], [4.0, 17.0], [4.0, 19.0],
           [5.0, 1.0], [5.0, 2.0], [5.0, 7.0], [5.0, 8.0], [5.0, 10.0], [5.0, 11.0], [5.0, 16.0], [5.0, 17.0], [5.0, 18.0], [5.0, 19.0], [5.0, 20.0],
           [6.0, 1.0], [6.0, 8.0], [6.0, 10.0], [6.0, 17.0], [6.0, 19.0],
           [7.0, 1.0], [7.0, 2.0], [7.0, 7.0], [7.0, 8.0], [7.0, 10.0], [7.0, 11.0], [7.0, 16.0], [7.0, 17.0], [7.0, 18.0], [7.0, 19.0], [7.0, 20.0],
           [8.0, 1.0], [8.0, 8.0], [8.0, 10.0], [8.0, 17.0], [8.0, 19.0],
           [9.0, 1.0], [9.0, 2.0], [9.0, 9.0], [9.0, 8.0], [9.0, 10.0], [9.0, 11.0], [9.0, 16.0], [9.0, 17.0], [9.0, 18.0], [9.0, 19.0], [9.0, 20.0],
           [10.0, 1.0], [10.0, 8.0], [10.0, 10.0], [10.0, 17.0], [10.0, 19.0],
           [11.0, 1.0], [11.0, 2.0], [11.0, 3.0], [11.0, 4.0], [11.0, 5.0], [11.0, 6.0], [11.0, 7.0], [11.0, 8.0], [11.0, 9.0], [11.0, 10.0], 
           [11.0, 11.0], [11.0, 12.0], [11.0, 13.0], [11.0, 14.0], [11.0, 15.0], [11.0, 16.0], [11.0, 17.0], [11.0, 18.0], [11.0, 19.0],
           [12.0, 1.0], [12.0, 8.0], [12.0, 10.0], [12.0, 17.0], [12.0, 19.0],
           [13.0, 1.0], [13.0, 2.0], [13.0, 3.0], [13.0, 4.0], [13.0, 5.0], [13.0, 6.0], [13.0, 7.0], [13.0, 8.0], [13.0, 9.0], [13.0, 10.0], 
           [13.0, 11.0], [13.0, 12.0], [13.0, 13.0], [13.0, 14.0], [13.0, 15.0], [13.0, 16.0], [13.0, 17.0], [13.0, 18.0], [13.0, 19.0]]

q1_task = [[3.0, 3.0], [3.0, 6.0], [3.0, 12.0], [3.0, 15.0],
           [5.0, 3.0], [5.0, 6.0], [5.0, 12.0], [5.0, 15.0],
           [7.0, 3.0], [7.0, 6.0], [7.0, 12.0], [7.0, 15.0],
           [9.0, 3.0], [9.0, 6.0], [9.0, 12.0], [9.0, 15.0]]

q1_wait = [[1.0, 21.0], [3.0, 21.0], [5.0, 21.0], [7.0, 21.0], [9.0, 21.0]]

# 第二象限数据（x坐标为负）
q2_path = [[-x, y] for x, y in q1_path]
q2_task = [[-x, y] for x, y in q1_task]
q2_wait = [[-x, y] for x, y in q1_wait]

# 第三象限数据（x和y坐标都为负）
q3_path = [[-x, -y] for x, y in q1_path]
q3_task = [[-x, -y] for x, y in q1_task]
q3_wait = [[-x, -y] for x, y in q1_wait]

# 第四象限数据（y坐标为负）
q4_path = [[x, -y] for x, y in q1_path]
q4_task = [[x, -y] for x, y in q1_task]
q4_wait = [[x, -y] for x, y in q1_wait]

# 合并所有象限的数据
all_path = q1_path + q2_path + q3_path + q4_path
all_task = q1_task + q2_task + q3_task + q4_task
all_wait = q1_wait + q2_wait + q3_wait + q4_wait

# 提取x和y坐标
path_x, path_y = zip(*all_path)
task_x, task_y = zip(*all_task)
wait_x, wait_y = zip(*all_wait)

# 绘制所有点（交换x和y坐标，使x轴朝上）
plt.scatter(path_y, path_x, c='blue', marker='o', s=20, alpha=0.7, label='Path point')
plt.scatter(task_y, task_x, c='red', marker='s', s=50, alpha=0.8, label='Task Point')
plt.scatter(wait_y, wait_x, c='green', marker='^', s=80, alpha=0.9, label='Waiting point')

# 添加坐标轴和网格
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
plt.grid(True, alpha=0.3)

# 设置坐标轴范围
plt.xlim(-25, 25)
plt.ylim(-15, 15)

# 设置坐标轴标签（交换x和y标签）
plt.xlabel('Y coordinate')
plt.ylabel('X coordinate')

# 添加图例
plt.legend(loc='upper right')

# 添加象限标注（调整位置）
plt.text(22, 7, 'The first quadrant', fontsize=12, ha='center', color='darkblue')
plt.text(22, -7, 'Second Quadrant', fontsize=12, ha='center', color='darkblue')
plt.text(-22, -7, 'The third quadrant', fontsize=12, ha='center', color='darkblue')
plt.text(-22, 7, 'Fourth Quadrant', fontsize=12, ha='center', color='darkblue')

# 调整布局
plt.tight_layout()
plt.show()