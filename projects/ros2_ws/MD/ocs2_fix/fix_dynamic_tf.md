# Claude Code 任务：用动态 TF 替代静态 world → base_link

## 背景
参考 ~/ros2_ws/MD/CLAUDE.md 、~/ros2_ws/MD/q&a.md。

完整工作循环： 

1. 阶段A：放置循环（输入传送带）  机器人驻车在输入传送带前  → 右臂先放托盘1到传送带（左臂等待）  → 左臂夹爪将托盘2给到右臂夹爪  → 左臂退出避让  → 右臂放托盘2到传送带  → 两个托盘都放完，左右手臂回到初始姿态 
2. 阶段B：平移到输出传送带  轮式底盘向左平移  → 到达输出传送带最佳取托盘位置 
3. 阶段C：抓取循环（输出传送带）  → 右臂夹爪抓取托盘1  → 左臂抓取右臂夹爪的托盘1  → 右臂抓取托盘2  → 返回阶段A 

机器人完整工作循环包含底盘移动（阶段B），所以不能用 static_transform_publisher 
发布 world → base_link。需要用 Isaac Sim 已有的 /odom 话题作为动态 TF 来源。

## 当前话题情况
Isaac Sim 已发布:
- `/odom` (nav_msgs/Odometry) — 底盘在世界坐标中的位姿
- `/tf` — Isaac Sim 的 publish_tf 节点发布的 TF（根 frame 名为 "Robot"）

## 修改文件
`~/ros2_ws/src/r1_description/launch/ocs2_isaac.launch.py`

## 修改内容

### 1. 删除之前添加的 static_tf_world 节点（如果存在的话）

删除或注释掉:
```python
    static_tf_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        ...
    )
```

### 2. 添加 odom_to_tf 桥接节点

Isaac Sim 发布的 /odom 消息包含 `header.frame_id`（通常是 "odom" 或 "world"）
和 `child_frame_id`（通常是 "base_link"）。

有两种方式把 /odom 转为 TF：

**方案A（推荐）：用 robot_localization 的 ekf_node**
如果项目中已经有 robot_localization 包，它会自动把 /odom 转为 TF。
但这个比较重，先用方案B。

**方案B：写一个轻量 odom_to_tf 节点**

在 `~/ros2_ws/src/r1_description/` 下创建文件 `scripts/odom_to_tf.py`:

```python
#!/usr/bin/env python3
"""
将 /odom (nav_msgs/Odometry) 转为 TF 广播: world → base_link
同时发布 world → odom 静态变换（假设 odom frame = world frame）
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # 发布 world → odom 静态变换 (identity)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        self.static_br.sendTransform(t)

        self.sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10)
        self.get_logger().info("odom_to_tf: publishing world → odom (static) + odom → base_link (dynamic)")

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        # 确保 frame_id 是 odom（不是其他名字）
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

然后给脚本加执行权限:
```bash
chmod +x ~/ros2_ws/src/r1_description/scripts/odom_to_tf.py
```

### 3. 在 CMakeLists.txt 中安装脚本

在 `~/ros2_ws/src/r1_description/CMakeLists.txt` 的 install 部分添加:
```cmake
install(PROGRAMS scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
```

如果 CMakeLists.txt 中还没有 scripts 目录相关的 install，检查是否有 
`install(DIRECTORY scripts/ ...)` 这种写法，有的话就不需要单独加了。

### 4. 修改 ocs2_isaac.launch.py

将之前的 static_tf_world 替换为 odom_to_tf 节点:

```python
    # ------------------------------------------------------------------
    # 7. Dynamic TF: world → odom → base_link
    #    从 Isaac Sim 的 /odom 话题读取底盘位姿，转为 TF
    #    支持阶段B底盘移动
    # ------------------------------------------------------------------
    odom_to_tf_node = Node(
        package="r1_description",
        executable="odom_to_tf.py",
        name="odom_to_tf",
        output="log",
        parameters=[{"use_sim_time": True}],
    )
```

保留 `static_tf_robot` 节点（Robot → base_link 桥接 Isaac Sim TF 树）。

在 `nodes` 列表中替换:
```python
    nodes = [
        planning_robot_state_publisher,
        odom_to_tf_node,       # ← 动态 TF (替代 static_tf_world)
        static_tf_robot,       # ← 保留 (桥接 Isaac Sim "Robot" frame)
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[ocs2_arm_controller_spawner]),
    ]
```

### 5. 处理潜在的 TF 冲突

Isaac Sim 的 publish_tf 可能也在发布 base_link 的子 frame TF。
如果出现 "TF_REPEATED_DATA" 警告，有两种处理方式:

- **优先方案**: 在 Isaac Sim 的 State_Telemetry_Graph 中，让 publish_tf 的
  Target Prim 只设为 `/World/Robot`（根级别），不让它发布 base_link 的子 TF。
  planning_robot_state_publisher 已经在做这件事了。

- **备选方案**: 删除 static_tf_robot 节点，让 Isaac Sim 的 "Robot" frame 独立存在，
  不与 planning_robot_state_publisher 的 TF 树连接。RViz Fixed Frame 设为 base_link 即可。

## 编译和验证

```bash
cd ~/ros2_ws
colcon build --packages-select r1_description
source install/setup.bash

# 启动 Isaac Sim → Play
# 确认 /odom 有数据:
ros2 topic echo /odom --once

# 启动 OCS2:
ros2 launch r1_description ocs2_isaac.launch.py

# 验证 TF 树完整性:
ros2 run tf2_ros tf2_echo world base_link
# 应该输出位姿数据且持续更新
```

## TF 树最终结构

```
world (固定)
  └── odom (静态 identity)
        └── base_link (动态, 来自 /odom)
              ├── trunk_link
              │     ├── left_base_link → left_Link1 ... left_Link7 → 左夹爪
              │     ├── right_base_link → right_Link1 ... right_Link7 → 右夹爪
              │     ├── head_camera_link → camera_H_link → 相机 frames
              │     └── body_link, waist_link
              ├── wheel_L_link, wheel_R_link
              └── caster_*_link

Robot (Isaac Sim 根 frame)
  └── base_link (静态 identity, 桥接用)
```

这样阶段A/C驻车时 odom → base_link 不变，阶段B移动时实时更新。
