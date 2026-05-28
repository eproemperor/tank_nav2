
更新地图信息  UpdateMapinfo
    初始化：哨兵出生点，敌方步兵出生点，补给区，中央蓝色区域，传送点，基地，迷宫
    每次轮询：更新敌人位置，更新哨兵位置

游戏是否开始  IfGameStart
    哨兵是否存在

重新映射速度  NavVelRemap
    设置新的速度上限

去目标点  GoBase
    输入：目标位置
    流程：前往下一个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）-->检测血量&子弹数量-->（前往补给区）-->寻找第二个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）
        -->前往传送点-->前往密 码发区-->前往传送点-->前往基地

开火  Fire

接收/发送密码  RSendPassward

