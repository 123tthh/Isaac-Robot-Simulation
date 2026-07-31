# 依赖恢复

`repositories.lock.yaml` 固定外部仓库提交。核心 OCS2/SIM1 只要求
`robot_descriptions`；IsaacLab-Arena、IsaacLab、Isaac-GR00T 和 Arena 内的
R1 实验描述均为可选组件。

核心恢复：

```bash
git clone https://github.com/fiveages-sim/robot_descriptions \
  projects/ros2_ws/src/robot_descriptions
git -C projects/ros2_ws/src/robot_descriptions checkout \
  fff4d08cf07cff966747986bbc01af75ff23e2ad
```

可选仓库有本地修改时，先 checkout 锁定提交，再应用
`reproducibility/patches/` 中的 tracked patch，最后解压对应的 untracked
archive。不要把嵌套 `.git` 目录复制进主仓库。
