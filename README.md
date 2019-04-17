# firefighter-robot-finished
2016级河南大学计算机与信息工程学院创新实践计划--消防机器人


各个包的功能介绍
flame_detection                 识别与定位

flame_detection_train           SVM支持向量机 图像识别的训练

achilles_base                   维护机器人坐标系的urdf文件。其中部分文件是由soildworks生成导出的。
                                执行其中roslaunch achilles_base display.launch后，在运行rviz，添加robotmodel后即可查看到机器人的仿真模型。

achilles_tf                     包括一个读取机器人可动关节的节点。和arduino_due驱动包achilles_base包配合使用。
                                
                                
achilles_sensor                 传感器，我们将一切和主控器相连接的设备称之为传感器，虽然他们本不该被命名为传感器。准确来说是外设。
                                这个包里面有摄像头，激光雷达，底盘，以及arduino_due控制板的ros驱动程序与设备规则，他们各自相互独立的放在不同包内。
                                注意，事实上achilles_sensor不是一个包，而它的子文件夹都是各个独立而不会互相影响的包。
                                
achilles_store                  存放一些自定义的msg和srv数据结构。配合整个机器系统

achilles_exploring              调用slam和move_base等算法包，同时保存了地图探索的代码，目前有些未知bug导致这部分功能暂时不能使用。

achilles_fighting               里面的launch文件会启动整个机器人的所有功能。里面包含了瞄准代码，这部分代码会控制水泵的开启和机器人舵机的旋转。

Arduino                         很明显，由于习惯用面向对象的方式编程，我使用了c++&arduino来做下位机。
                                其中唯一的.ino文件就是下位机的代码。
                                libary里面是各个库文件，库文件中包含了设备管理，multcode协议等，与achilles_sensor/serial中的多个.py文件相对应。
                                注意，achilles_sensor/serial也含有这些库文件。如果需要.py源码，请去achilles_sensor/serial中查看。
                                如果需要c++源码，请在本文件libary里面查看。
                                


----本文件由 河南大学 网络工程专业 吴雪铭 所拥有。如有需要请联系本人 wechat账号 K15837821186。 e-mail: 1362385699@qq.com
