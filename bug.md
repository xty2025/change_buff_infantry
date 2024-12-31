1. measure出现nan，怀疑是solvepnp/asin/atan2出问题（这些地方都未做检查） 罕见
2. optimize error 10 较常见                
3. optimize 中途出现nan （传入正常） 罕见 
4. 使用predict冲突 出现memory错误 常见
5. driver 出问题 较常见
6. yaw -170 -> 170 应该柔和过渡 常见
7. 运行时间长时段错误 较常见
8. 莫名奇妙的卡顿/卡死
**注释掉predictions.push_back(car->getPredictResult(timestamp, carid));没有任何任何任何问题！！！！！！！！**
**原来是CppAD库是非线程安全的，禁止两个线程同时使用CppAD库（非同时使用可以）（Dvector的创建甚至都会导致问题）**
**更进一步的测试表明solve和Dvector的创建会导致cppad崩溃，而简单的dvector同时创建不会导致问题，但是实现创建好Dvector后再resize或者赋值都没关系**