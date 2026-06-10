## 代码使用指南

架构基本参考25哨兵代码：

`img_process`:图像处理&定位，识别得到图中相关信息，通过自定义消息发布到
/map_info topic上

MapInfo.msg：

is_exist 该单位是否存在；

is_out_of_center 该单位是否在中心区域范围外

pos 该单位的x、y、theta信息

MapInfoMsgs.msg：

map_info 各个单位的MapInfo.msg信息，枚举量参考img_process.hpp中

enemy_num 当前场上的敌人数量

sentry_hp 当前哨兵血量（double型）

is_transfering 哨兵是否在传送门位置附近

is_bullet_low 当前子弹是否低于一定阈值

需要注意的是enemy的position只会输出当前离哨兵最近的那一个，基地前的红色敌人不考虑在内

`robot_bring_up`:总启动包，所有的参数和launch文件都在这里。sentry.launch.py是总启动launch文件，
bringup_launch.py是nav2的总启动launch文件

`behavior_tree`:行为树，有三个，这里拆分到三个文件夹中（哨兵的是放到一起了便于管理）。
`robot_behavior_tree`包内放自定义行为树节点插件；
`robot_bt_decision_maker`负责加载所有的自定义插件和行为树xml，串联所有节点；
`robot_bt_navigator` nav2框架相关内容，调用自带导航插件