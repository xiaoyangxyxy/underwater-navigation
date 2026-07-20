# 管道几何声纳约束有效性验证

## 实验设置

- 实验名称：管道几何声纳约束有效性验证
- Baseline 方法：IMU + Depth + DVL + MAG
- Proposed 方法：IMU + Depth + DVL + MAG + Sonar
- 运行时间：69.5 s
- 样本数：279
- random_seed：42
- 管道半径：2.5 m
- 轨迹类型：6DOF sinusoidal pipe trajectory

## 主要结论

- rmse_position 从 0.11539 m 降低到 0.05193 m
- mean_position_error 从 0.10319 m 降低到 0.04545 m
- max_position_error 从 0.19287 m 降低到 0.10446 m
- final_position_error 从 0.16388 m 降低到 0.06315 m
- rmse_y 从 0.10352 m 降低到 0.00841 m
- Sonar 对横向 Y 漂移抑制最明显

## 说明

- 本实验用于验证管道几何 SonarFactor 的有效性
- 本实验没有加入 Camera marker factor
- 本实验没有修改 IMU / Depth / DVL / MAG factor
- Baseline 和 Baseline + Sonar 使用相同轨迹、相同随机种子、相同运行时长
