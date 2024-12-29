# AutoAim - Infantry
version: 0.0.1 (unfinished)
## 依赖
- opencv
- openvino
- eigen
- spdlog
- GXIAPI
- cppad & ipopt (cpp自动微分优化库)
## 编译
```shell
mkdir build && cd build
cmake ..
make
```
## 运行
```shell
bash utils/scripts/startGTK.sh # 还未迁移至仓库
cd bin && sudo ./autoaim_infantry
```

## 代码结构

```
.
├── modules/                # 核心模块
│   ├── interfaceType.hpp   # 接口定义
│   ├── modules.hpp         # 模块公共头文件
│   ├── driver/             # 硬件驱动
│   ├── detector/           # 目标检测
│   ├── solver/             # 弹道解算
│   ├── controller/         # 控制系统
│   ├── tracker/            # 目标追踪
│   └── predictor/          # 运动预测
├── utils/                  # 工具组件
│   ├── scripts/            # 脚本文件
│   ├── include/            # 工具头文件
│   └── models/             # 模型文件
├── AutoAim.cpp             # 主程序
├── CMakeLists.txt          # CMake配置文件
└── config.json             # 配置文件
```

## 开发进度
### 功能实现
- [x] NUC上编译  
- [x] NUC上运行  
- [x] 对车控制  
- [ ] 打弹  
- [ ] 反陀螺  
### 算法实现
- [ ] 匹配和分配序号算法  
- [ ] 预测器初始化函数  
- [ ] 火控逻辑编写  
- [ ] JSON配置  
- [ ] 合适的日志输出  
- [ ] udpsender编写
- [ ] 代码文档  
### 模块检查
- [x] driver  
- [x] detector  
- [ ] solver  
- [ ] controller  
- [ ] tracker  
- [ ] predictor  

## Feature
### 工程特色
- 模块间解耦  
所有模块间无法通信，不可调用，只可通过回调函数的方式进行通信。提供给上级接口为纯虚函数接口，由下级模块提供实现（工厂模式）。  
- 语法特性  
使用lambda表达式，STL容器，智能指针等特性。**不使用模板😂。**
- 伪代码
```cpp
// 初始化各个模块
driver = new Driver()        // 相机和串口驱动
controller = new Controller()// 控制系统  
detector = new Detector()    // 目标检测
tracker = new Tracker()      // 目标跟踪
solver = new Solver()        // 弹道解算
predictor = new Predictor()  // 运动预测

// 配置硬件
...

// 注册回调函数
controller.registPredictFunc(predictor.predictFunc)
controller.registSolveFunc(solver.camera2worldFunc)
driver.registReadCallback(callback=[](serialData) {
    result = controller.control(serialData)
    driver.sendSerial(result)
})

// 启动线程
driver.run()

// 主循环
while(true) {
    if(!driver.hasNewFrame())
        continue
        
    frame = driver.getLatestFrame()
    imu = driver.getMatchedIMU(frame.timestamp)
    
    if(!lastDetected || timeout)
        tracker.merge(detector.detect(frame))
        
    if(lastDetected) {
        predictions = predictor.predict(deltaTime)
        projects = solver.world2camera(predictions, imu)
        rois = tracker.calcROI(projects)
        
        for(roi : rois)
            tracker.merge(detector.detect(frame, roi))
            
        tracker.update(projects)
    } else {
        tracker.update()
    }
    
    predictor.update(solver.camera2world(tracker.getResult()))
    lastDetected = tracker.isDetected()
}
```

### 算法特色
(待补充)