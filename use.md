<!-- 先打静止装甲板，调准 camera_offset。
再打旋转小符，通过 COMPANSATE_TIME 找到提前量。
最后通过 COMPANSATE_PITCH/YAW 进行微小的现场环境修正。 -->
## 新加可视化部分：
<!-- 1. 基础检测层 (Detector)
这是模型最原始的输出：

黄色矩形框：检测到的装甲板边缘。AutoAim.cpp:292
黄色文字标签：显示 armor id (数字识别结果) 和 s (置信度分值)。AutoAim.cpp:294
蓝色大矩形框：由 YOLO 检测到的整车区域。AutoAim.cpp:300
蓝色文字标签：整车的 car id 和 score。AutoAim.cpp:301
2. 追踪与解算层 (Tracker & Solver)
反映了系统在空间中“锁定”目标的状态：

青色实心圆 (Cyan)：当前正在追踪的装甲板中心点。AutoAim.cpp:341
垂直引导线：从装甲板中心点延伸到地面 (
Z=0) 的直线，用于直观感受目标的离地高度。AutoAim.cpp:347
红色/黄色文字：在装甲板旁显示该目标的唯一跟踪 ID 以及解算出的 yaw 角度。AutoAim.cpp:350-353
3. 运动预测层 (Predictor)
展示算法对敌人下一步位置的预判：

灰色圆点：预测出的敌方底盘中心位置。AutoAim.cpp:416
绿色圆圈：预测出的击打点（即目标在子弹飞行一段时间后的装甲板位置）。AutoAim.cpp:424
红色小圆圈：如果预测的装甲板当前处于不可见状态（遮挡），则显示红色。AutoAim.cpp:421
4. “小地图”俯视图 (Mini-map)
代码中逻辑将 3D 分量映射到了屏幕坐标（约在图像中间 500x500 区域）：

红色/白色圆点与短线：在俯视图上标注出的敌人位置及其车头朝向。AutoAim.cpp:356-363
系统中心准星：在图像正中心 (500, 500) 绘制的绿色十字架，代表枪管的基准瞄准点。AutoAim.cpp:446-447 -->

# aim_request 的数值含义
1：常规自瞄（打装甲板）。
2：小能量机关（打符）。
3：打哨兵/特定中心模式。
4：大能量机关。
0：无请求。

---

## 调试模式切换 (Replayer & Driver)
若要切换回放模拟与实车模式，请修改 [AutoAim.cpp](AutoAim.cpp) 第 70 行附近：

### 实车模式 (Normal Mode)
```cpp
auto driver = createDriver(); 
// auto driver = createReplayer(...);
```

### 回放调试模式 (Debug Mode)
```cpp
// auto driver = createDriver(); 
auto driver = createReplayer("../record/video/20260227150320.avi", "../record/serial/20260227150320.txt", false, 1.0);
```
**说明：**
- 第一个参数为 `.avi` 视频文件路径。
- 第二个参数为 `.txt` 串口同步数据路径。
- 第三个参数为 `onlyVideo` (bool)，设为 `false` 会回放当时 IMU 和电控指令数据。
- 第四个参数为 `speed` (float)，回放倍速。
