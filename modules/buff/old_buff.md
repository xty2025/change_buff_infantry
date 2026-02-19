'''
模型（red_buff / pose 任务）返回的主要输出是一个形状类似 (1, N, D) 的数组（你之前看到过 shape [1,300,24]）。每条检测记录包含 24 个元素，表示：bbox + score + class + 6 个关键点（每个关键点 3 值）。
每条检测（D=24）的字段顺序（推断并与脚本处理一致）
bbox（4）: cx, cy, w, h （中心点 + 宽高；可能是归一化到 0..1，也可能是像素坐标）
score（1）: 置信度得分
class（1）: 类别索引（整数）
keypoints（6 × 3 = 18）: for each of 6 keypoints -> [x, y, conf]（x,y 可能归一化或像素，脚本通过值范围判断）
（4 + 1 + 1 + 18 = 24，和你看到的 24 对应）
# NCHW、NHWC两种数据格式。
'''

'''
关键数学（简要）
新大符的目标函数改为速度曲线（角速度作为目标），具体形式为：

spd(t) = a * sin(ω * t) + b

其中：
- spd 的单位为 rad/s，t 的单位为 s；
- 参数范围：a ∈ [0.780, 1.045]，ω ∈ [1.884, 2.000]；
- b 始终满足 b = 2.090 − a（由规则给定）。

说明与与原流程的兼容性：
- 原先在文档中使用的是角度模型 angle(t) = −A * cos( ω (t + φ) ) + B * t + C。
	如果把角速度 spd(t) 对时间积分，就能得到角度的表达形式：

	angle(t) = ∫ spd(t) dt = −(a/ω) * cos(ω * t) + b * t + C0

	若需包含相位偏移 φ，可写为 spd(t) = a * sin(ω * (t + φ)) + b，积分后得到：

	angle(t) = −(a/ω) * cos(ω * (t + φ)) + b * t + C0

	因此新的速度目标函数与原来的角度模型在形式上是等价的（通过积分关联），只不过参数的物理含义更直接：`a` 是速度振幅，`ω` 是角频率，`b` 是速度的直流分量（由 a 约束）。

参数估计：
- 建议直接用测得的角速度样本（若能获得）用 RANSAC + Ceres 最小二乘拟合 `a, ω, φ, C0`（其中 C0 为积分常数，可由初始角度确定）。
- 如果只有角度样本，也可以用原来方式拟合角度模型，但在拟合时应将参数约束为满足 b=2.090−a 的关系，或先对角度做数值微分得到速度样本再拟合速度模型以提高鲁棒性。

子弹抛物线求解（在 getPitchYawFromRobotCoor）保持不变：
- 目标在机器人坐标 (x, y, z)，水平距离 horizontal = sqrt(x^2 + z^2)（单位 mm，代码乘/除 1e-3 切换 m）。
- 将发射角 tanθ 作为未知，代入二维弹道方程并变换为关于 t = tanθ 的二次方程：

	a * t^2 + b * t + c = 0 （代码中 a = -0.5 * g * horizontal^2 / v^2， b = horizontal， c = a + target.y*1e-3）

- 取二次方程的第二个根（.second），pitch = atan(result)（弧度转角度）。yaw = -atan2(x, z)（转为角度并加上 AFTER_YAW 补偿）。

坐标变换与后续流程保持不变：
- world2Camera：通过 solvePnP(worldPoints, cameraPoints) 得到 rVec/tVec，构造齐次矩阵 w2c。
- camera2Gimbal：用预设或参数化 tVec 构造 4x4 变换。
- gimbal2Robot：由 pitch/yaw/roll 构造旋转矩阵（代码中 roll 未纳入实际影响矩阵乘法）。
子弹抛物线求解（在 getPitchYawFromRobotCoor）：
目标在机器人坐标 (x, y, z)，水平距离 horizontal = sqrt(x^2 + z^2)（单位 mm，代码乘/除 1e-3 切换 m）。
将发射角 tanθ 作为未知，代入二维弹道方程并变换为关于 t = tanθ 的二次方程：
a * t^2 + b * t + c = 0 （代码中 a = -0.5 * g * horizontal^2 / v^2， b = horizontal， c = a + target.y*1e-3）
取二次方程的第二个根（.second），pitch = atan(result)（弧度转角度）。yaw = -atan2(x, z)（转为角度并加上 AFTER_YAW 补偿）。
坐标变换：
world2Camera：通过 solvePnP(worldPoints, cameraPoints) 得到 rVec/tVec，构造齐次矩阵 w2c。
camera2Gimbal：用预设或参数化 tVec 构造 4x4 变换。
gimbal2Robot：由 pitch/yaw/roll 构造旋转矩阵（代码中 roll 未纳入实际影响矩阵乘法）。
'''