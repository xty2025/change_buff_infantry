# namespace::xty::
# How to collaborate by git;
# cd workspace
# git init (git status查看是否保存)(git remote show origin查看链接的哪个仓库信息)
# (git remote -v)
# git add.
# git commit -m "change"
# git remote add origin  https://github.com/xty2025/change_buff_infantry.git
# git push -u origin main(推送到main分支)(main可换成分支名)
# git pull origin main 拉取并合并到本地的最新代码
# 如果有冲突：git add <conflicted_file>
# 解决后再提交：git commit -m "Merge remote-tracking branch 'origin/main' into main"
# git pull origin develop(添加到其他分支)
# 新建一个分支：
# git checkout -b new-branch-name
# git pull origin main
# git pull origin develop(分支仓库)
# git fetch origin (只拉去，不合并)
# git branch -M main 新创一个分支
# git push -u origin main提交到新的分支




# 速度目标函数为：spd = a ∗ sin(𝜔𝜔 ∗
# 𝑡𝑡) + 𝑏𝑏，其中 spd 的单位为 rad/s，t 的单位为 s，a 的取值范围为 0.780~1.045，ω的取值范围为
# 1.884~2.000，b 始终满足 b=2.090-a。
# 比赛开始及比赛开始 1 分 30 秒时，双方各获得 1 次激活小能量机关的机会，该机会可累积。
# 小能量机关进入正在激活状态 20 秒后，若其仍未被激活，则将恢复为未激活状态。每次45s.
# 比赛开始 3 分钟、4 分 15 秒、5 分 30 秒时三次机会。
# 大能量机关进入正在激活状态 20 秒后，若其仍未被激活，则将恢复为未激活状态。

'''
    当大能量机关进入正在激活状态，能量机关随机点亮 5 块装甲模块中的任意 2 个，被点亮的装甲模块亮
    起特殊灯效，并且该装甲模块对应的灯臂中轴有箭头状流动灯效。此时，若弹丸在 2.5 秒内击中任意一个
    被点亮的装甲模块，该灯臂会改变相应灯效，此后 1 秒内，机器人可以击中另一个被点亮的装 甲模块，不
    论是否成功，能量机关都将重新随机点亮 5 块装甲模块中的任意 2 个，以此类推。
'''
''' 这样的话，可以每次打一个，只是环数少一点 '''


## 0/1/2 个 Buff 目标下的检测、解算、连续双击算法（xty）

### 1) 目标约束
- 单帧目标数只允许三种情况：`0` / `1` / `2`。
- 若模型输出超过 2 个候选，按置信度降序保留前 2 个。
- 若为 0 个，直接进入失效保护，不发有效击打角。

### 2) 检测层（BuffDetector）
- 遍历 OpenVINO 输出张量，每 `24` 个 float 解析一个候选：
    - `[0:4]` 框坐标
    - `[4]` 置信度
    - `[5]` 类别
    - `[6:24]` 六个关键点（每点 x,y,score）
- 过滤规则：
    - `confidence >= 0.8`
    - 框和关键点均在合法图像区域
- 结果排序：按置信度降序排序，只保留前 2 个。
- 每个候选独立做 `change_scale`（letterbox 坐标 -> 原图坐标）后，生成一个 `Armor`。

### 3) 解算层（BuffCalculator）
- 对“被选中的一个目标”输入 6 个点：`R, C, U, Rgt, D, L`。
- 通过现有 `matrixCal -> angleCal -> directionCal -> predict` 输出当前目标的预测 `pitch/yaw`。
- 若本帧是双目标场景，则由决策层决定当前使用第 1 目标还是第 2 目标进解算。

### 4) 决策层（快速切枪状态机）

定义状态：
- `SINGLE_TRACK`：单目标跟踪
- `DOUBLE_CYCLE`：双目标连续双击窗口

状态转移逻辑：
- 检测数 = 0：
    - 进入失效保护，输出无效角（`+90` 哨兵值），不执行双击窗口。
- 检测数 = 1：
    - 强制 `SINGLE_TRACK`，目标索引固定 `0`。
- 检测数 = 2：
    - 首次进入时开启 `DOUBLE_CYCLE`，记录 `cycle_start`。
    - 默认先瞄 `primary(0)`。
    - 当检测到“首发已发射”后，**立即**切换到 `secondary(1)`，并保持连发。
    - `首发已发射` 判据：
        1. 优先：串口回传 `available_shoot_number` 下降（更接近真实发射事件）；
        2. 兜底：首发 `shoot_flag=1` 指令发出后立即切换，避免回传延迟造成慢切。
    - 超过 `1.0s` 窗口后重置为新一轮双击周期。

> 说明：该策略按“发射事件”触发切枪，满足“枪口发射一颗弹丸之后立马调转并连发”。

### 5) 容错与保护
- `cameraPoints.size() != 6` 时直接视作本帧失败，输出无效角。
- 双目标切换时若某帧降为 1 目标，立即退回 `SINGLE_TRACK`。
- 若持续 0 目标，保持失效保护直到重新检测到目标。

### 6) 与当前代码对应关系
- 检测：`modules/buff/BuffDetector.cpp`
    - 新增多目标解析与上限 2 个候选保留。
    - 新增接口：按索引获取目标关键点。
- 主流程：`AutoAim.cpp`
    - Buff 分支中按检测数 0/1/2 进入不同决策路径。
    - 增加 1 秒快速切枪窗口，实现连续双击。

### 7) 建议实测调参项
- `confidence_thres`：默认 `0.8`。
- `first_shot_hold`：默认 `0.25s`。
- `second_shot_window`：默认 `0.75s`。
- 需要结合云台响应速度、发弹延时、弹速波动进行联调。