# Ros Programming
Platform: Ubuntu 16.04
ROS Version: ros-kinetic
Author: Chen YuHong

## 第一部分：文件目录
——Folder 2017011507_陳昱宏_Ros Programming  
——————2017011507_陳昱宏_Ros Programming.pdf  
——————ReadMe.txt  
——————Folder husky_rcl_controller  
——————————Folder include  
——————————————Folder husky_rcl_controller  
——————————————————HuskyController.hpp  
——————————————————HuskyRosApplication.hpp  
——————————Folder src  
——————————————main.cpp  
——————————————HuskyController.cpp  
——————————————HuskyRosApplication.cpp  
——————————CMakeLists.txt  
——————————package.xml  
——————Folder husky_rcl_controller_srv  
——————————Folder include  
——————————————Folder husky_rcl_controller_srv  
——————————Folder src  
——————————Folder srv  
——————————————husky_rcl_controller_srv.srv  
——————————CMakeLists.txt  
——————————package.xml  
## 第二部分：编译执行
将文件夹中的husky_rcl_controller和husky_rcl_controller_srv放到本地的catkin工作空间下的src目录，执行以下指令即可编译，  
`$ catkin build husky_rcl_controller`  
编译后，利用rosrun命令可以运行该节点，  
`$ rosrun husky_rcl_controller husky_rcl_controller`  

## 第三部分：各个源码文件的作用
（一）husky_rcl_controller文件夹内：  
1.HuskyController.hpp:  
    是算法类，定义了控制器的类内成员和函数，用来对外部指令进行处理，控制机器人，采用PID控制器。  
2.HuskyController.cpp:  
    实现算法类的成员函数，包括更新当前位置、更新目标位置、PID控制与参数设置等。  
3.HuskyRosApplication.hpp:  
    是节点类，定义了husky_rcl_controller节点的类内成员和函数，主要用来和外界交互，接收和发送Topic以及和Service相通信。  
4.HuskyRosApplication.cpp:  
    实现节点类的成员函数，包括Service的回调函数、Topic的回调函数和控制指令的发送。  
5.main.cpp:  
    主函数。  
（二）husky_rcl_controller_srv文件夹内：  
1.husky_rcl_controller_srv.srv:  
    定义了Service的Request和Response，格式如下：  
	`float64 targetX`  
	`float64 targetY`  
	`float64 targetOrientationZ.`  
	`—`  
	`bool result`  
	`loat64 timeCost`  

## Reference:  
<https://blog.csdn.net/u010647296/article/details/83143946?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task>  
<https://www.jianshu.com/p/ed1d9a490bd1>  
<https://github.com/leggedrobotics/ros_best_practices>  
<https://blog.csdn.net/lin5103151/article/details/89893380>  
