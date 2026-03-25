# 大符移动补偿标定说明

## 目的

`BuffShootingTableCalibration` 只负责大符运动补偿标定，输出配置写入：

- `config.buff.buff_shooting_table_calib`

它和下面两条链路独立：

- 打车装甲板标定：`ShootingTableCalibration` -> `config.controller.shoot_table_adjust`
- 静止符基础弹道标定：`BuffStaticShootingTableCalibration` -> `config.buff.shoot_table_adjust`

大符标定时，程序会先沿用 `config.buff.shoot_table_adjust` 作为基础弹道补偿，再让你采集“大符周期运动残差”，最终只拟合大符移动补偿。

这版没有沿用打车装甲板的二阶多项式，而是改成了更适合圆周运动的周期模型：

```text
offset = intercept
       + coef_sin * sin(theta)
       + coef_cos * cos(theta)
       + coef_sin2 * sin(2theta)
       + coef_cos2 * cos(2theta)
       + coef_dist * distance
       + coef_height * height
```

- `theta`：当前解算出的预测旋转角
- `distance`：待击打点水平距离，单位 m
- `height`：待击打点高度，单位 m

## 配置

在 [config.json](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/config.json) 的 `buff` 段新增：

```json
"buff_shooting_table_calib": {
  "enable": false,
  "apply_on_big_buff_only": true,
  "pitch": {
    "intercept": 0.0,
    "coef_sin": 0.0,
    "coef_cos": 0.0,
    "coef_sin2": 0.0,
    "coef_cos2": 0.0,
    "coef_dist": 0.0,
    "coef_height": 0.0
  },
  "yaw": {
    "intercept": 0.0,
    "coef_sin": 0.0,
    "coef_cos": 0.0,
    "coef_sin2": 0.0,
    "coef_cos2": 0.0,
    "coef_dist": 0.0,
    "coef_height": 0.0
  }
}
```

## 启动

直接运行独立脚本：

```bash
./utils/scripts/start_big_buff_moving_compensation_calib.sh
```

脚本位置：[utils/scripts/start_big_buff_moving_compensation_calib.sh](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/start_big_buff_moving_compensation_calib.sh)

兼容入口仍保留：

- [utils/scripts/start_buff_shooting_table_calib.sh](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/start_buff_shooting_table_calib.sh)

## 操作

- `t`：开始或恢复大符跟踪
- `空格`：打一小段射击脉冲
- `w/s`：上/下调 pitch
- `a/d`：左/右调 yaw
- `/`：把当前 `buff.buff_shooting_table_calib` 直接套上预览
- `c`：当前弹着点修正满意后保存一条样本
- `r`：停止跟踪并清零修正
- `q`：退出

程序会持续检测大符装甲板，并实时刷新基准解算角。这里的基准角已经包含：

- 原始解算
- `config.buff.shoot_table_adjust` 的基础弹道补偿

你手动调的 `pitch/yaw` 只表示“大符运动额外残差”。

## 输出

样本保存到 `record/big_buff_moving_compensation_*.csv`，字段包括：

- `rotation_angle`
- `horizontal_distance`
- `height`
- `relative_yaw`
- `relative_pitch`
- `target_x/y/z`
- `absolute_yaw`
- `absolute_pitch`

## 拟合

采完数据后执行：

```bash
python3 utils/scripts/solverFit/fit_big_buff_moving_compensation.py record/big_buff_moving_compensation_xxx.csv
```

脚本位置：[utils/scripts/solverFit/fit_big_buff_moving_compensation.py](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/solverFit/fit_big_buff_moving_compensation.py)

兼容入口仍保留：

- [utils/scripts/solverFit/fit_buff_shoot_table.py](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/solverFit/fit_buff_shoot_table.py)

输出会直接给出一段可粘到 `config.json` 的 `buff_shooting_table_calib` 配置，同时打印 pitch/yaw 的拟合误差。
