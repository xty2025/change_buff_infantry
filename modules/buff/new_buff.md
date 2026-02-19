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