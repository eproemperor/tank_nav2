"condition":
游戏是否开始  IfGameStart
    哨兵是否存在

"action":
更新地图信息  UpdateMapinfo
    初始化：哨兵出生点，敌方步兵出生点，补给区，中央蓝色区域，传送点，基地，迷宫
    每次轮询：更新敌人位置，更新哨兵位置



重新映射速度  NavVelRemap
    设置新的速度上限

去目标点  GoBase
    输入：目标位置
    流程：前往下一个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）-->检测血量&子弹数量-->（前往补给区）-->寻找第二个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）
        -->前往传送点-->前往密 码发区-->前往传送点-->前往基地

开火  Fire

接收/发送密码  RSendPassward


#
##
以下是 `nav_msgs/OccupancyGrid` 消息结构的表格：

## nav_msgs/OccupancyGrid 消息结构

| 字段路径 | 类型 | 说明 |
|---------|------|------|
| **header** | std_msgs/Header | 消息头信息 |
| header.stamp | builtin_interfaces/Time | 时间戳 |
| header.stamp.sec | int32 | 秒 |
| header.stamp.nanosec | uint32 | 纳秒 |
| header.frame_id | string | 坐标系名称（如 "map"） |
| **info** | nav_msgs/MapMetaData | 地图元数据 |
| info.map_load_time | builtin_interfaces/Time | 地图加载时间 |
| info.map_load_time.sec | int32 | 秒 |
| info.map_load_time.nanosec | uint32 | 纳秒 |
| info.resolution | float32 | 地图分辨率（米/像素） |
| info.width | uint32 | 地图宽度（像素数） |
| info.height | uint32 | 地图高度（像素数） |
| **origin** | geometry_msgs/Pose | 地图原点在地球坐标系中的位姿 |
| origin.position | geometry_msgs/Point | 原点位置 |
| origin.position.x | float64 | X 坐标 |
| origin.position.y | float64 | Y 坐标 |
| origin.position.z | float64 | Z 坐标 |
| origin.orientation | geometry_msgs/Quaternion | 原点朝向（四元数） |
| origin.orientation.x | float64 | X 分量 |
| origin.orientation.y | float64 | Y 分量 |
| origin.orientation.z | float64 | Z 分量 |
| origin.orientation.w | float64 | W 分量 |
| **data** | sequence&lt;int8&gt; | 栅格数据，一维数组，每个元素表示该单元格状态 |

## data 字段取值说明

| 值 | 含义 |
|----|------|
| -1 | 未知区域（未探索） |
| 0 | 空闲（可通行） |
| 100 | 占用（障碍物） |
| 0-100 之间 | 占用概率（越高越可能被占用） |



#
以下是三类节点的**必须实现**和**可选实现**的成员函数清单：

## 📊 三类节点函数对比表

| 函数类型 | SyncActionNode | ConditionNode | AsyncActionNode |
|---------|----------------|---------------|-----------------|
| **构造函数** | ✅ 必须 | ✅ 必须 | ✅ 必须 |
| **providedPorts()** | ✅ 必须 | ✅ 必须 | ✅ 必须 |
| **tick()** | ✅ 必须 | ✅ 必须 | ✅ 必须 |
| **halt()** | ❌ 可选 | ❌ 可选 | ✅ **强烈推荐** |

---

## 1️⃣ **BT::SyncActionNode** (同步动作节点)

### 必须实现的成员

```cpp
class MyAction : public BT::SyncActionNode
{
public:
    // 1️⃣ 构造函数 (必须)
    MyAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    
    // 2️⃣ 声明端口 (必须)
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("input_data"),
            BT::OutputPort<bool>("output_result")
        };
    }
    
    // 3️⃣ tick() 函数 (必须) - 返回 SUCCESS 或 FAILURE
    BT::NodeStatus tick() override {
        // 执行动作，立即返回结果
        return BT::NodeStatus::SUCCESS;  // 或 FAILURE
    }
};
```

### 可选的辅助函数

```cpp
class MyAction : public BT::SyncActionNode
{
public:
    // 可选：在树创建时调用一次
    virtual void initialize() {}
    
    // 可选：在树销毁时调用
    virtual void terminate() {}
    
    // 可选：获取节点名称
    std::string name() const { return _name; }
    
    // 可选：获取黑板引用
    BT::Blackboard::Ptr blackboard() { return config().blackboard; }
};
```

### 可用的基类方法（在 tick() 中使用）

```cpp
BT::NodeStatus tick() override {
    // ✅ 读取输入端口
    auto input = getInput<std::string>("input_data");
    if (!input) {
        throw BT::RuntimeError("Missing input");
    }
    std::string value = input.value();
    
    // ✅ 读取带默认值
    auto timeout = getInput<double>("timeout", 1.0);
    
    // ✅ 写入输出端口
    setOutput("output_result", true);
    
    // ✅ 获取配置
    const auto& cfg = config();
    
    // ✅ 获取黑板
    auto blackboard = cfg.blackboard;
    
    // ✅ 获取ROS节点（如果存储在黑板中）
    auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    return BT::NodeStatus::SUCCESS;
}
```

---

## 2️⃣ **BT::ConditionNode** (条件节点)

### 必须实现的成员

```cpp
class MyCondition : public BT::ConditionNode
{
public:
    // 1️⃣ 构造函数 (必须)
    MyCondition(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    
    // 2️⃣ 声明端口 (必须)
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("threshold", 10),  // 带默认值
            BT::InputPort<std::string>("topic_name")
        };
    }
    
    // 3️⃣ tick() 函数 (必须) - 只返回 SUCCESS 或 FAILURE
    BT::NodeStatus tick() override {
        // 检查条件
        auto threshold = getInput<int>("threshold", 10);
        
        if (check_condition()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
    
private:
    bool check_condition() { return true; }
};
```

### ConditionNode 特殊说明

```cpp
// ⚠️ ConditionNode 不应该返回 RUNNING
// ❌ 错误
BT::NodeStatus tick() override {
    return BT::NodeStatus::RUNNING;  // 不允许！
}

// ✅ 正确
BT::NodeStatus tick() override {
    return condition ? SUCCESS : FAILURE;
}
```

---

## 3️⃣ **BT::AsyncActionNode** (异步动作节点)

### 必须实现的成员

```cpp
class MyAsyncAction : public BT::AsyncActionNode
{
public:
    // 1️⃣ 构造函数 (必须)
    MyAsyncAction(const std::string& name, const BT::NodeConfig& config)
        : BT::AsyncActionNode(name, config)
        , _started(false)
        , _halt_requested(false) {}
    
    // 2️⃣ 声明端口 (必须)
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("duration", 1.0)
        };
    }
    
    // 3️⃣ tick() 函数 (必须) - 可返回 RUNNING
    BT::NodeStatus tick() override {
        if (_halt_requested) {
            _halt_requested = false;
            _started = false;
            return BT::NodeStatus::FAILURE;
        }
        
        if (!_started) {
            // 首次调用，开始异步任务
            _started = true;
            start_async_task();
            return BT::NodeStatus::RUNNING;
        }
        
        // 检查任务是否完成
        if (is_task_complete()) {
            _started = false;
            return _task_success ? SUCCESS : FAILURE;
        }
        
        // 仍在执行中
        return BT::NodeStatus::RUNNING;
    }
    
    // 4️⃣ halt() 函数 (强烈推荐实现)
    void halt() override {
        _halt_requested = true;
        cancel_async_task();
    }
    
private:
    bool _started;
    bool _halt_requested;
    bool _task_success;
    
    void start_async_task() {}
    bool is_task_complete() { return true; }
    void cancel_async_task() {}
};
```

### AsyncActionNode 状态机示例

```cpp
class NavigateToPose : public BT::AsyncActionNode
{
public:
    BT::NodeStatus tick() override {
        switch (_state) {
            case State::IDLE:
                // 开始导航
                send_navigation_goal();
                _state = State::NAVIGATING;
                return BT::NodeStatus::RUNNING;
                
            case State::NAVIGATING:
                // 检查导航状态
                if (is_navigation_complete()) {
                    _state = State::IDLE;
                    return BT::NodeStatus::SUCCESS;
                }
                if (is_navigation_failed()) {
                    _state = State::IDLE;
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::FAILURE;
    }
    
    void halt() override {
        cancel_navigation();
        _state = State::IDLE;
    }
    
private:
    enum class State { IDLE, NAVIGATING };
    State _state = State::IDLE;
};
```

---

## 📋 快速选择指南

| 场景 | 推荐节点类型 | 原因 |
|------|-------------|------|
| 读取传感器、更新状态 | `SyncActionNode` | 快速返回，不耗时 |
| 条件判断（有无敌人、血量） | `ConditionNode` | 语义清晰，只返回成功/失败 |
| 导航、等待、长时间动作 | `AsyncActionNode` | 可返回 RUNNING，支持中断 |
| 开火（单次） | `SyncActionNode` | 立即完成 |
| 持续开火（连发） | `AsyncActionNode` | 需要 RUNNING 状态 |

---

## 🔧 完整注册示例

```cpp
// 所有节点注册在同一个文件中
BT_REGISTER_NODES(factory)
{
    // 注册同步动作
    factory.registerNodeType<MyAction>("MyAction");
    
    // 注册条件节点
    factory.registerNodeType<MyCondition>("MyCondition");
    
    // 注册异步动作
    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");
}
```

---

## ⚠️ 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `ConditionNode` 返回 RUNNING | ConditionNode 不允许 RUNNING | 改为返回 SUCCESS/FAILURE |
| `SyncActionNode` 耗时太长 | 阻塞行为树 | 改用 `AsyncActionNode` |
| `AsyncActionNode` 没有 `halt()` | 无法中断任务 | 实现 `halt()` 方法 |
| `providedPorts()` 是 `static` | 忘记加 static | 必须加 `static` 关键字 |


