#### Move to Goal Control
######  这个程序应用了蒙特卡洛定位 [amcl](http://wiki.ros.org/amcl?distro=lunar) ，参考论文"Multi-Robot Assignment and Formation Control"5.1.2.3节实现了一个系统稳定性控制。效果是稳定地控制小车到达目标位置（无路径规划、避障能力）

#### 使用教程
1. 启动turtlebot
> roslaunch turtlebot3_bringup turtlebot3_robot.launch

2. 启动rviz
>rosrun rviz rviz -d `rospack find turtlebot3_location`/rviz/turtlebot3_locat.rviz

3. 启动move_to_goal节点（里面包含了amcl定位）
>roslaunch turtlebot3_location move_to_goal.launc
