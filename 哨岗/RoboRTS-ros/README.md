# T-DT人工智能挑战赛项目

本项目是东北大学T-DT战队基于Robomaster官方机器人平台所编写的机器人间全自动对抗射击项目，内容主要哨岗定位及决策等.

------

## 项目文件作用说明

- **roborts_camera** 

  1. 相机标定

  2. 打开两个相机，获取图像，发送给robots_detection

  3. 保存比赛视频

- **roborts_common**
  此文件夹包含机器人算法中常用的计算数据的数学函数，包括计算两点间距离，多点排序，对数函数计算，斜率计算，3/2变换（三维空间到二维空间），2/3变换（二维空间到一维空间）等

  

- **roborts_costmap**

  1. 加成惩罚区(非实体障碍块)信息在static_layer.cpp中.惩罚区标记为致命障碍,为了保证机器人活动范围又不误入惩罚区,更改inflation_layer.cpp取消该障碍区的默认膨胀层,并在static_layer.cpp中指定膨胀半径.加成惩罚区状态可以根据场上信息实时变化.

  2. 在test_costmap.cpp中记录了应用于雷达识别的地图划分情况,在匹配成功后发布目标点信息.为了防止误识别导致行为混乱,通过实验将不必要的区域信息置为负数.以供决策判断.
     启动节点:
     雷达识别的节点是

     ```
       <!-- 雷达识别 -->
       <node pkg="roborts_costmap" type="test_costmap" name="lidar_detection_node" respawn="false" />
     ```

- **roborts_decision**

  1. 通过blackboard读取来自感知和导航等节点的信息,behavior_test.cpp对行为进行切换.
     针对不同情况,执行的行为包括一下11个
     补弹addbullet_behavior
     补血addhp_behavior
     主动追击 attack_behavior 
     撤离加成区 back_boot_area_behavior
     反击 counter_behavior
     雷达识别 lidardetected_behavior
     追踪 mychase_behavior 
     射击 shoot_behavior 
     支援 support_behavior
     巡逻 mypatrol_behavior
     逃跑 myescape_behavior

  2. 机器人之间以及和哨岗的通讯在robot_communicate.h

  3. referee_system.h用于仿真时代替裁判系统发布当前场上信息.

     

- **roborts_detecton**

  1. 前置相机 接收roborts_camera 发送的图像，从图像中识别出敌方机器人的装甲板信息，控制云台对装甲版进行打击

  2. 后置相机 接收roborts_camera发送的图像，从图像中识别到视觉标签的信息，将视觉标签的数据发送给定位，辅助定位系统


- **udp_socket**
  实现机器人与哨岗的无线通讯

- **beginner_tutorials**
  哨岗初始化校正，通过该功能包校正相机安装的位置和角度误差

------

## 设备驱动

- **USB绑定**
  NUC通过USB口获取雷达、相机、下位机的数据.因此在无法读取相关设备信息的时候需要查看USB口是否正常连接.如果没有则需要进行USB绑定,步骤如下:

  1. 首先查看待绑定的USB口号,和计算机上需要绑定的位置.
  2. 根据相应信息更改/etc/udev/rule.d 中相应的配置文件.

  

- **相机驱动安装声明**
  哨岗中使用了海康威视的工业相机，需要安装其SDK才能完成正常运行，下载链接如下:
  https://www.hikvision.com/cn/download_more_403.html
  在相机使用中，需要对SDK相关内容进行改写，将相机调用等程序添加到roborts_camera中（上传代码中已完成此部分），实现驱动程序与ROS的耦合

------

## 效果说明

在项目根目录中有图片效果展示

------

## 项目运行

打开相机

```
sudo su 
rosrun roborts_camera roborts_camera_node
```
识别节点

```
rosrun roborts_detection armor_detection_node
rosrun roborts_detection armor_detection_node1
```








