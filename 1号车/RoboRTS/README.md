# T-DT人工智能挑战赛项目   
   
本项目是东北大学T-DT战队基于Robomaster官方机器人平台所编写的机器人间全自动对抗射击项目，内容主要包含全场定位、自主导航、计算机视觉识别、自主决策、哨岗定位及决策等.   
   
------   
   
## 项目文件作用说明   
   
- **roborts_bringup**   
  |── launch 相关代码的launch启动文件   
  |  └── mapping_hokuyo.launch 北阳激光雷达运行文件，启动雷达后可以自行建图，保存到maps文件夹中   
  |  └── roborts_navigation.launch 机器人导航算法启动文件，包含运行定位启动命令，雷达启动命令，通讯串口启动命令等   
  |  └── roborts_socket.launch 机器人间通讯启动文件，实现机器人间信息交流   
  |  └── stm32bringup_icra.launch 机器人底层控制板与上位机间信息交流启动文件，同时实现上位机向下位机数据传送，下位机向上位机数据传送的功能，主要作为调试时使用   
  |  └── roborts_simulate.launch 机器人仿真启动文件，包含机器人导航算法命令，定位算法命令，模拟tf数算法命令等   
  |  └── static_tf.launch 虚拟tf数变化启动文件   
  |── maps   
  |  └── icra2020.pgm 地图图片文件，可以插入到rviz中作为虚拟地图   
  |  └── icra2020.yaml 地图参数文件，包括建图时地图边界尺寸等相关参数   
  |── rviz   
  |  └── roborts.rviz 启动rviz程序的相关参数，包括rviz插件信息，尺寸设置，话题接收等   
  |── scripts   
  |── worlds   
  |  └── icra2020.world 包括地图障碍物，尺寸等相关信息，在建图过程中自动生成   
   
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
   
- **roborts_localization**   
   
  roborts_localization   
  |── config   
  |  └── localization.yaml  #Localization参数配置文件，需要在launch file中load   
  |── re_vi   
  |  └── re_vi.txt               #包含视觉标签定位信息的配置文件   
  |── localization_config.h  #Localization参数类   
  |── localization_math.cpp     
  |── localization_math.h    #模块内通用数学函数   
  |── localization_node.cpp  #主节点和Main函数   
  |── localization_node.h       
  |──re_vi.h                         #加载视觉标签信息    
  |── log.h                  #Glog Wrapper   
  |── package.xml   
  |── types.h                #Type define   
  |── amcl                   #AMCL算法代码目录   
  |  |── amcl_config.h      #AMCL参数类   
  |  |── amcl.cpp           #AMCL主要逻辑代码   
  |  |── amcl.h   
  |  |── CMakeLists.txt       
  |  |── config   
  |  |  └── amcl.yaml      #AMCL参数配置文件   
  |  |── map   
  |  |  |── amcl_map.cpp      
  |  |  └── amcl_map.h     #AMCL地图运算相关代码   
  |  |── particle_filter    #粒子滤波器相关代码   
  |  |  |── particle_filter.cpp   
  |  |  |── particle_filter_gaussian_pdf.cpp   
  |  |  |── particle_filter_gaussian_pdf.h   
  |  |  |── particle_filter.h   
  |  |  |── particle_filter_kdtree.cpp   
  |  |  |── particle_filter_kdtree.h   
  |  |  └── particle_filter_sample.h   
  |  └── sensors            #里程计模型与激光雷达传感器模型   
  |  |  |── sensor_laser.h   
  |  |  |── sensor_laser.cpp   
  |  |  |── sensor_odom.h   
  |  |  └── sensor_odom.cpp   
   
- **roborts_planning**   
  a_star_planner.cpp:   
   
  1. 在使用A*算法进行寻路计算过程中,将父节点与目标点距离的影响放入通过优化估值函数减少了对冗余点的计算.   
   
  2. 目标点被占用A*算法会将当前情况判定为GP_GOAL_INVALID_ERROR.并无法规划路径.我们将分为两种情况解决.   
     (1)如果目标点在加成惩罚区,为了防止将加成区误判为不可达,将地图中部点(大概率必经点)作为临时目标点进行路径规划,以达到探索目的.   
     (2)目标点不在加成惩罚区,则在目标点周围8个点中找一个可到达的点进行路径规划.   
   
  global_planner_node.cpp   
   
  1. 机器人被卡住(或者误入障碍层)无法移动时.增加节点状态STUCK.并在local_planner_node.cpp中执行反向速度指令.   
     运行的节点包括   
   
     ```   
     <!-- 全局路径规划-->   
       <node pkg="roborts_planning" type="global_planner_node" name="global_planner_node" respawn="false" />   
       <!-- 局部路径规划 -->   
       <node pkg="roborts_planning" type="local_planner_node" name="local_planner_node" respawn="false" />   
     ```   
   
- **serial**   
  包括与下位机的串口协议、位姿的解算、加成区精准对位解算。通过该功能包接收来自下位机的信号，并将解算后的云台、底盘信号发送至下位机。   
   
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
  机器人中使用了海康威视的工业相机，需要安装其SDK才能完成正常运行，下载链接如下:   
  https://www.hikvision.com/cn/download_more_403.html   
  在相机使用中，需要对SDK相关内容进行改写，将相机调用等程序添加到roborts_camera中（上传代码中已完成此部分），实现驱动程序与ROS的耦合   
   
------   
   
## 效果展示   
   
在项目根目录中有图片效果展示   
   
------   
   
## 项目运行   
   
打开相机   
   
```   
sudo su    
rosrun roborts_camera roborts_camera_node   
```   
   
其余所有节点   
   
```   
roslaunch roborts_bringup roborts_winner.launch    
```   
   
   
   
   
   
   
   