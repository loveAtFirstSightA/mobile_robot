# AMR 状态信息

# amr id
uint8 amr_id

# 状态机
string state # 运行状态: 'idle', 'run', 'pause' 等

# 当前位姿
amr_msgs/Pose pose

# 电池电量
float32 battery

