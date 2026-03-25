# 静止符射表标定说明

## 目的

`BuffStaticShootingTableCalibration` 用于给静止符或近似静止符采集基础弹道误差，最终拟合到：

- `config.buff.shoot_table_adjust`

这部分和打车的 `controller.shoot_table_adjust` 是同一类二阶模型，但配置完全独立，不会改动车的拟合结果。

建议流程：

1. 先做这一套静止符基础弹道标定。
2. 把结果写入 `config.buff.shoot_table_adjust`。
3. 再做大符移动补偿标定，把大符运动残差单独拟合到 `config.buff.buff_shooting_table_calib`。

## 模型

```text
offset = intercept
       + coef_z * z
       + coef_d * d
       + coef_z2 * z^2
       + coef_zd * z * d
       + coef_d2 * d^2
```

- `z`：目标高度，单位 m
- `d`：水平距离，单位 m

## 启动

```bash
./utils/scripts/start_buff_static_shooting_table_calib.sh
```

脚本位置：[utils/scripts/start_buff_static_shooting_table_calib.sh](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/start_buff_static_shooting_table_calib.sh)

## 操作

- `t`：开始或恢复跟踪
- `空格`：打一小段射击脉冲
- `w/s`：上/下调 pitch
- `a/d`：左/右调 yaw
- `/`：套用当前 `buff.shoot_table_adjust`
- `c`：保存当前样本
- `r`：停止跟踪并清零修正
- `q`：退出

## 输出

输出 CSV 位于：

- `record/buff_static_shooting_table_*.csv`

字段包括：

- `z_height`
- `horizontal_distance`
- `relative_yaw`
- `relative_pitch`
- `target_x/y/z`
- `absolute_yaw`
- `absolute_pitch`

## 拟合

采完数据后执行：

```bash
python3 utils/scripts/solverFit/fit_buff_static_shoot_table.py record/buff_static_shooting_table_xxx.csv
```

脚本位置：[utils/scripts/solverFit/fit_buff_static_shoot_table.py](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/solverFit/fit_buff_static_shoot_table.py)

输出可直接粘到 `config.buff.shoot_table_adjust`。
