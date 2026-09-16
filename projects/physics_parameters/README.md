# Physics parameter experiments / 物理参数实验

Optional research scripts, separate from the R1 teaching/recording runtime.
本模块用于场景调整及托盘释放位置、质量、摩擦参数研究，不是主采集入口。

- `search_v0/`–`search_v8/`: historical search revisions / 历次参数搜索版本。
- `scene_tools/`: scene adjustment scripts / 场景调整脚本。
- `results/`: retained reference results and summary script / 已有参考结果与汇总。

The former top-level Chinese directory is now `projects/physics_parameters/`.
`SEARCH*`, `pos`, `result` were renamed to the directories above. Local `.venv/`
content is ignored and must be recreated after moving the project.

Paths resolve by finding `project_manifest.yaml` above the script or current
working directory, or using `ISAAC_OCS_PROJECT_ROOT`. Before opening an external
Isaac Script Editor session, launch from the project root with:

```bash
export ISAAC_OCS_PROJECT_ROOT="$PWD"
./run.sh start-isaac
```

In Script Editor load the desired script from this directory. Scripts with a
shared helper use the helper beside that revision. Results use this module's
`results/` directory, not an author's Desktop. Running an experiment may overwrite
same-named results; preserve reference results first. Existing scene prim paths
such as `/World/Pallet` refer to USD nodes and are intentionally unchanged.

从项目根目录设置环境变量后启动，脚本编辑器才能在没有 `__file__` 时可靠定位工程。
各版本的辅助脚本从同目录加载；结果不再写入桌面。研究脚本依赖特定场景节点，
此次只完成路径解析与语法检查，未重新运行物理搜索，也未新增采集。
