# Buff 标定总览

当前仓库内和 buff/打车相关的标定链路一共 3 条，互相独立：

## 1. 打车装甲板基础弹道标定

- 程序：[ShootingTableCalibration.cpp](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/ShootingTableCalibration.cpp)
- 输出配置：`config.controller.shoot_table_adjust`
- 说明文档：[ShootingTableCalibration_README.md](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/ShootingTableCalibration_README.md)

这部分未改动，继续沿用你原来的打车拟合结果。

## 2. 静止符基础弹道标定

- 程序：[BuffStaticShootingTableCalibration.cpp](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/BuffStaticShootingTableCalibration.cpp)
- 启动脚本：[utils/scripts/start_buff_static_shooting_table_calib.sh](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/start_buff_static_shooting_table_calib.sh)
- 拟合脚本：[utils/scripts/solverFit/fit_buff_static_shoot_table.py](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/solverFit/fit_buff_static_shoot_table.py)
- 输出配置：`config.buff.shoot_table_adjust`
- 说明文档：[BuffStaticShootingTableCalibration_README.md](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/BuffStaticShootingTableCalibration_README.md)

这部分用于先把符的基础弹道误差拟合掉，逻辑类比打车的 `shoot_table_adjust`，但配置独立放在 `buff` 段。

## 3. 大符移动补偿标定

- 程序：[BuffShootingTableCalibration.cpp](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/BuffShootingTableCalibration.cpp)
- 启动脚本：[utils/scripts/start_big_buff_moving_compensation_calib.sh](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/start_big_buff_moving_compensation_calib.sh)
- 拟合脚本：[utils/scripts/solverFit/fit_big_buff_moving_compensation.py](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/utils/scripts/solverFit/fit_big_buff_moving_compensation.py)
- 输出配置：`config.buff.buff_shooting_table_calib`
- 说明文档：[BuffShootingTableCalibration_README.md](/home/user/桌面/xty_merge/change_buff_infantry-merge_buff/BuffShootingTableCalibration_README.md)

这部分只拟合大符自身的周期运动残差。运行时先应用 `config.buff.shoot_table_adjust`，再额外叠加 `config.buff.buff_shooting_table_calib`。
