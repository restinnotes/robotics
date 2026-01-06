# Git Pull 实时进度查看指南

## 为什么 git pull 看起来"没效果"？

1. **默认输出不详细**：`git pull` 默认只显示最终结果，不显示下载进度
2. **网络慢时看起来像卡住**：大文件或慢网络时，没有进度条
3. **已经是最新**：如果本地已经是最新的，git pull 会快速完成但看起来"没做任何事"

## 查看实时进度的方法

### 方法 1：使用 `--progress` 和 `--verbose` 参数

```bash
# 显示详细的进度信息
git pull --progress --verbose

# 或者分两步执行，先 fetch 看进度
git fetch --progress --verbose
git merge origin/main  # 或 git rebase origin/main
```

### 方法 2：使用 `GIT_CURL_VERBOSE` 环境变量

```bash
# 显示 HTTP 传输的详细信息
GIT_CURL_VERBOSE=1 GIT_TRACE=1 git pull
```

### 方法 3：分步执行（推荐）

```bash
# 步骤 1：先 fetch，可以看到下载进度
git fetch --progress origin

# 步骤 2：查看有什么更新
git log HEAD..origin/main --oneline

# 步骤 3：合并或变基
git merge origin/main
# 或
git rebase origin/main
```

### 方法 4：使用 `--stat` 查看变更统计

```bash
git pull --stat
```

### 方法 5：实时监控网络活动

在另一个终端窗口运行：
```bash
# 监控网络流量
watch -n 1 'netstat -i | grep -E "Iface|eth0|wlan0"'
```

## 当前仓库状态

你的本地分支领先远程 1 个提交，但远程有 4 个新提交。

建议操作：
```bash
# 1. 先查看远程更新
git fetch --progress --verbose

# 2. 查看差异
git log HEAD..origin/main --oneline

# 3. 合并或变基（根据你的工作流选择）
git pull --rebase  # 使用 rebase 保持历史线性
# 或
git pull           # 使用 merge 创建合并提交
```

## 检查是否真的在更新

```bash
# 查看远程分支信息
git ls-remote origin

# 查看本地和远程的差异
git log --oneline --graph --all --decorate -10
```
