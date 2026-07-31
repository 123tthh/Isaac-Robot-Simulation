# 可复现迁移制品

该目录保存不适合直接进入主 Git 历史的复现元数据：

- `patches/`：嵌套仓库的 tracked binary patch。
- `artifacts/`：未跟踪源码、SIM1 数据或 Docker 镜像归档；大型归档被
  `.gitignore` 排除，只提交其 SHA-256。
- `manifests/`：场景、资源、数据、源码补丁和镜像校验清单。

正式迁移时应同时携带主 Git、Git LFS 对象和 manifests 中列出的外部制品。
