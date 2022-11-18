<!--
 * @FileName: 
 * @Description: 
 * @Autor: Liujunjie/Aries-441
 * @StudentNumber: 521021911059
 * @Date: 2022-11-13 22:12:15
 * @E-mail: sjtu.liu.jj@gmail.com/sjtu.1518228705@sjtu.edu.cn
 * @LastEditTime: 2022-11-18 13:12:05
-->
需要四个终端
终端1：ros2 launch tribot view_tribot_world.launch.py
将小车生成在一个空的世界中。

终端2：ros2 launch tribot  set_kinematic.launch.py
小车的运动学模型节点，根据控制输入和运动学模型，输出在gazebo中的位置
键盘控制实现后还有部分函数内部的问题未解决
还有部分

终端3：ros2 run tribot tribot_teleop
键盘控制运动节点
注意是run，而非launch，原文件大改后相关launch文件没有改好

终端4：rviz或者其他可视化节点