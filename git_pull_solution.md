# Git Pull 问题解决方案

## 当前情况

- **本地分支**：领先 origin/main 1 个提交
- **远程分支**：有 4 个新提交（本地没有）
- **状态**：分支已偏离（diverged）

## 解决方案

### 方案 1：使用 Rebase（推荐，保持历史线性）

```bash
# 查看实时进度
git pull --rebase --progress --verbose
```

这会：
1. 先拉取远程更新
2. 把你的本地提交"重新应用"到远程最新提交之上
3. 保持提交历史线性

### 方案 2：使用 Merge（创建合并提交）

```bash
# 查看实时进度
git pull --no-rebase --progress --verbose
```

这会：
1. 拉取远程更新
2. 创建一个合并提交
3. 保留完整的分支历史

### 方案 3：先查看再决定

```bash
# 1. 查看远程更新（带进度）
git fetch --progress --verbose

# 2. 查看差异
git log --oneline --graph --all --decorate -10

# 3. 查看本地未提交的更改
git status

# 4. 根据情况选择合并方式
git pull --rebase --progress    # 或
git pull --no-rebase --progress
```

## 实时查看进度的方法

### 方法 1：使用详细输出
```bash
git pull --rebase --progress --verbose
```

### 方法 2：分步执行
```bash
# 步骤 1：fetch（可以看到下载进度）
git fetch --progress --verbose origin

# 步骤 2：查看更新了什么
git log HEAD..origin/main --oneline

# 步骤 3：合并
git rebase origin/main  # 或 git merge origin/main
```

### 方法 3：使用环境变量显示更多信息
```bash
GIT_TRACE=1 GIT_CURL_VERBOSE=1 git pull --rebase --progress
```

## 配置默认行为（避免每次都提示）

```bash
# 设置默认使用 rebase
git config pull.rebase true

# 或设置默认使用 merge
git config pull.rebase false

# 或只允许快进合并（更安全）
git config pull.ff only
```

## 检查进度的小技巧

在另一个终端窗口运行：
```bash
# 监控 git 进程
watch -n 0.5 'ps aux | grep git'

# 或监控网络流量
iftop -i eth0  # 需要安装 iftop
```
