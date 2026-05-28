task:
    里面识别和其他部分内容已经给大家写好了，需要大家补的是自定义规划器、行为树、串口的内容，readme里面有介绍。

#
##
5.25
    串口框架搭建，serialpro学习
    剩余：writer注册与消息回调，串口发送函数

##
5.26
    maybe I finished this task,but there still has problems.What worse is that I cloud not even check it.
    problems:1.change position
             2.check callback function,the passward has two different type so needs to be transformed.

##
5.27
###
1️⃣Learning BT_tree knowledge and what is a planer.
        黑板是 BehaviorTree.CPP 库提供的内置类
        ReactiveSequence
```xml
<ReactiveSequence>
    <Action ID="UpdateMapinfo"/>
    <Condition ID="IfGameStart"/>
    <Action ID="NavVelRemap"/>
    <Action ID="GoBase"/>
</ReactiveSequence>
```
        执行特点：
        ✅ 每次 tick 都重新检查所有节点、
        ✅ 如果任何子节点返回 FAILURE，立即中断并返回 FAILURE、
        ✅ 适合实时响应的场景（如检测到游戏结束立刻停止）
###
2️⃣Write down BT_tree methods,make sure each part's task.
3️⃣tomorrow task:1.learn how to write a action an condition,
                2.setup template,finished each .h