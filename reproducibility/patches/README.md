# 嵌套仓库补丁

`*.tracked.patch` 由各嵌套仓库固定 commit 上的 `git diff --binary` 生成。
应用前必须先 checkout `dependencies/repositories.lock.yaml` 中对应 commit。
