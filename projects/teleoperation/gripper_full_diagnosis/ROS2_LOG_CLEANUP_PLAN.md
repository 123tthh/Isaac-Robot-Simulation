# ros2_log 备份与整理计划

生成时间：2026-05-28 14:47-14:50  
目标目录：`/home/gtk/ros2_log`  
当前工程目录：`/home/gtk/teleoperation/sim1`

## 1. 已完成备份

已先做完整压缩备份，未删除原始文件。

```text
备份文件: /tmp/ros2_log_backup_20260528_144757.tar.gz
原始大小: 193M
压缩大小: 21M
归档条目: 199
sha256: 916bb054fd444f69203a47f00520b536d322c64747cd03c56d3fa5d9b224eb9e
```

恢复命令：

```bash
tar -xzf /tmp/ros2_log_backup_20260528_144757.tar.gz -C /home/gtk
```

## 2. 当前目录概况

`/home/gtk/ros2_log` 当前约：

```text
文件数: 175
目录数: 24
大小: 193M
```

主要内容类型：

- 根目录历史日志：`log*.md`、`*_gripper*.log`、若干零散 `.md/.py/.urdf`
- `scripts/`：长期保留的脚本、报告、调试记录
- `scripts/夹爪调试/`：夹爪控制核心历史与当前版本
- `scripts/Graph/`：Isaac Sim Action Graph 导出
- `gripper_full_diagnosis/`：2026-05-28 场景/夹爪完整诊断导出
- `gripper_data/`：PGIA 早期数据采样
- `test script/`：临时测试脚本与 pycache
- `urdf_source_probe/`：URDF 来源探测输出

## 3. 发现的完全重复文件

按 SHA256 内容完全相同：

```text
/home/gtk/ros2_log/gripper_full_diagnosis/ALL_Graph.md
/home/gtk/ros2_log/scripts/Graph/ALL_Graph.md

/home/gtk/ros2_log/gripper_full_diagnosis/scene_structure.md
/home/gtk/ros2_log/scripts/Graph/scene_structure.md

/home/gtk/ros2_log/scripts/夹爪调试/effort control/诊断/check_pgia_usd_symmetry.py
/home/gtk/ros2_log/scripts/夹爪调试/夹爪参数检查/check_pgia_usd_symmetry.py

/home/gtk/ros2_log/ros2_state_chain_graph.md
/home/gtk/ros2_log/scripts/ros2_state_chain_graph.md

/home/gtk/ros2_log/gripper_full_diagnosis/Gripper_Control_Graph.md
/home/gtk/ros2_log/scripts/Graph/Gripper_Control_Graph.md
```

建议保留规则：

- Graph 导出统一保留在 `scripts/Graph/`。
- 完整诊断包统一保留在 `gripper_full_diagnosis/`，但若只保留一份，优先保留 `scripts/Graph/` 中的图类导出。
- 重复脚本 `check_pgia_usd_symmetry.py` 保留在 `scripts/夹爪调试/夹爪参数检查/`。
- `ros2_state_chain_graph.md` 保留在 `scripts/`。

## 4. 最大冗余来源

大文件主要集中在根目录历史运行日志：

```text
58M  /home/gtk/ros2_log/left_gripper_v38.log
34M  /home/gtk/ros2_log/right_gripper_force.log
21M  /home/gtk/ros2_log/left_gripper_effort.log
20M  /home/gtk/ros2_log/right_gripper_v38.log
16M  /home/gtk/ros2_log/left_gripper_v37.log
14M  /home/gtk/ros2_log/right_gripper_v41.log
12M  /home/gtk/ros2_log/left_gripper_force.log
7.6M /home/gtk/ros2_log/script_right_interface_debug.log
6.7M /home/gtk/ros2_log/left_gripper_v41.log
3.1M /home/gtk/ros2_log/right_gripper_v37.log
```

这些适合归档到 `archive/runtime_logs/`，不建议直接删除，至少保留当前 v4.1 的左右日志用于修夹爪。

## 5. 推荐整理结构

建议将 `/home/gtk/ros2_log` 整理成：

```text
ros2_log/
  README.md
  active/
    gripper_v4_1/
      script_gripper_v4.1.py
      left_gripper_v41.log
      right_gripper_v41.log
      latest_notes.md
  scripts/
    graph/
    ocs2/
    teaching/
    camera/
    robot_model/
    gripper/
      effort_control_history/
      effort_pos_current/
      diagnostics/
      usd_symmetry/
  reports/
    ocs2_control_pipeline/
    gripper_diagnosis/
    handover_teaching/
  data/
    gripper_runtime/
    urdf_source_probe/
  archive/
    root_logs_20260509_20260522/
    runtime_logs/
    obsolete_duplicates/
    test_scratch/
```

## 6. 删除/移动分级策略

### A. 可安全移动到 archive

这些不应直接删除，但可以移动：

- 根目录 `log*.md`
- 根目录历史大 `.log`，除了当前正在用的 `left_gripper_v41.log`、`right_gripper_v41.log`
- 根目录旧版夹爪 effort/v32/v33/v35/v36/v37/v38 日志
- `test script/`
- 零散 `1.md`、`2.md`、`3.md`、`check.md`、`data.md`

### B. 可去重删除候选

仅在确认保留副本后删除：

```text
/home/gtk/ros2_log/gripper_full_diagnosis/ALL_Graph.md
/home/gtk/ros2_log/gripper_full_diagnosis/Gripper_Control_Graph.md
/home/gtk/ros2_log/gripper_full_diagnosis/scene_structure.md
/home/gtk/ros2_log/scripts/夹爪调试/effort control/诊断/check_pgia_usd_symmetry.py
/home/gtk/ros2_log/ros2_state_chain_graph.md
```

### C. 必须保留

当前修夹爪直接相关：

```text
/home/gtk/ros2_log/scripts/夹爪调试/effort+pos/script_gripper_v4.1.py
/home/gtk/ros2_log/left_gripper_v41.log
/home/gtk/ros2_log/right_gripper_v41.log
/home/gtk/ros2_log/scripts/夹爪调试/effort+pos/setup_v4_hybrid.py
/home/gtk/ros2_log/scripts/夹爪调试/effort+pos/script_gripper_v4.0.py
/home/gtk/ros2_log/scripts/夹爪调试/effort+pos/script_gripper_effort_position_v4_1.py
/home/gtk/ros2_log/scripts/夹爪调试/USD左右夹爪一致性检查/
/home/gtk/ros2_log/scripts/夹爪调试/夹爪参数检查/
/home/gtk/ros2_log/scripts/Graph/
/home/gtk/ros2_log/gripper_full_diagnosis/
```

## 7. 建议执行顺序

1. 保留当前 `/tmp/ros2_log_backup_20260528_144757.tar.gz`，必要时复制到持久目录。
2. 生成 `ros2_log/README.md`，说明目录规范。
3. 新建 archive 目录。
4. 先移动根目录历史日志到 archive，不删除。
5. 再处理完全重复文件。
6. 最后删除 `__pycache__`/`.pyc` 这类可再生成文件。

## 8. 待用户确认的破坏性操作

以下操作需要确认后执行：

```text
1. 是否允许移动根目录历史日志到 /home/gtk/ros2_log/archive/runtime_logs/
2. 是否允许删除 5 组完全重复文件中的冗余副本
3. 是否允许删除 test script/__pycache__/rebuild_gripper_effort_graph.cpython-310.pyc
4. 是否允许将零散根目录 md/py/urdf 按类别移动到 scripts/reports/data/archive
```

建议先执行“移动归档，不删除内容”；待 Claude 修夹爪完成后，再清理旧版大日志。
