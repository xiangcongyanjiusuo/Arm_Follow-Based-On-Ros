# 🚀 Linux环境下 Claude Code + DeepSeek V4-Pro 完整配置指南

> **适用系统**：Ubuntu 20.04+ / Debian 10+  
> **最后更新**：2026-05-22  
> **版本**：DeepSeek V4-Pro 满血版 + 最大思考强度

---

## 📋 目录

1. [环境要求](#1-环境要求)
2. [安装 Node.js](#2-安装-nodejs)
3. [安装 Claude Code](#3-安装-claude-code)
4. [获取 DeepSeek API Key](#4-获取-deepseek-api-key)
5. [配置 DeepSeek V4-Pro](#5-配置-deepseek-v4-pro)
6. [验证配置](#6-验证配置)
7. [VSCode 远程 SSH 连接](#7-vscode-远程-ssh-连接)
8. [常用命令速查](#8-常用命令速查)
9. [故障排除](#9-故障排除)

---

## 1️⃣ 环境要求

### 系统要求
- ✅ Ubuntu 20.04 LTS 或更高版本（推荐 22.04/24.04）
- ✅ 内存：至少 4GB（推荐 8GB+）
- ✅ 磁盘空间：至少 2GB 可用空间
- ✅ 网络：能访问 `api.deepseek.com` 和 `deb.nodesource.com`

### 前置检查
```bash
# 检查系统版本
cat /etc/os-release | grep PRETTY_NAME

# 检查是否已安装 Node.js
node --version 2>/dev/null || echo "未安装"

# 检查 SSH 服务状态
systemctl status ssh | grep "Active:"
```

---

## 2️⃣ 安装 Node.js

### 方法一：使用 NodeSource 官方源（推荐）

```bash
# 添加 NodeSource 仓库（安装 Node.js 20.x LTS）
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -

# 安装 Node.js
sudo apt install nodejs -y

# 验证安装
node --version    # 应显示 v20.x.x
npm --version     # 应显示 10.x.x
```

### 方法二：使用 NVM（可选，适合需要多版本管理）

```bash
# 安装 NVM
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash

# 重新加载 shell 配置
source ~/.bashrc  # 或 source ~/.zshrc

# 安装 Node.js 20 LTS
nvm install 20

# 验证
node --version
```

---

## 3️⃣ 安装 Claude Code

### 全局安装

```bash
# 使用 npm 全局安装 Claude Code
sudo npm install -g @anthropic-ai/claude-code

# 验证安装
claude --version    # 应显示类似 2.1.x
```

### 国内镜像加速（如网络慢）

```bash
# 使用淘宝镜像安装
sudo npm install -g @anthropic-ai/claude-code --registry=https://registry.npmmirror.com
```

---

## 4️⃣ 获取 DeepSeek API Key

### 注册并创建 API Key

1. **访问 DeepSeek 开放平台**：
   ```
   https://platform.deepseek.com
   ```

2. **注册/登录账号**
   - 支持手机号、邮箱注册
   - 支持微信/支付宝充值

3. **创建 API Key**：
   - 登录后进入「API 密钥」页面
   - 点击「创建 API 密钥」
   - 复制生成的密钥（格式：`sk-xxxxxxxx`）
   - ⚠️ **密钥只显示一次，请妥善保存**

---

## 5️⃣ 配置 DeepSeek V4-Pro

### 创建配置目录

```bash
mkdir -p ~/.claude
```

### 创建配置文件

使用你喜欢的编辑器创建 `~/.claude/settings.json`：

```bash
nano ~/.claude/settings.json
```

**粘贴以下完整配置**（替换 `YOUR_API_KEY` 为你的真实密钥）：

```json
{
  "env": {
    "ANTHROPIC_AUTH_TOKEN": "sk-你的DeepSeek_API_Key",
    "ANTHROPIC_BASE_URL": "https://api.deepseek.com/anthropic",
    "ANTHROPIC_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_OPUS_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_SONNET_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_HAIKU_MODEL": "deepseek-v4-flash",
    "CLAUDE_CODE_SUBAGENT_MODEL": "deepseek-v4-flash",
    "CLAUDE_CODE_EFFORT_LEVEL": "max"
  },
  "permissions": {
    "allow": [],
    "deny": []
  }
}
```

### 配置说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `ANTHROPIC_AUTH_TOKEN` | `sk-xxxxx` | 你的 DeepSeek API Key |
| `ANTHROPIC_BASE_URL` | `https://api.deepseek.com/anthropic` | DeepSeek API 端点 |
| `ANTHROPIC_MODEL` | `deepseek-v4-pro` | 主力模型（最强） |
| `ANTHROPIC_DEFAULT_OPUS_MODEL` | `deepseek-v4-pro` | Opus 角色模型 |
| `ANTHROPIC_DEFAULT_SONNET_MODEL` | `deepseek-v4-pro` | Sonnet 角色模型 |
| `ANTHROPIC_DEFAULT_HAIKU_MODEL` | `deepseek-v4-flash` | Haiku 快速模型 |
| `CLAUDE_CODE_SUBAGENT_MODEL` | `deepseek-v4-flash` | 子代理模型（轻量任务） |
| `CLAUDE_CODE_EFFORT_LEVEL` | `max` | 最大思考强度 🔥 |

### 一键配置命令（复制即用）

```bash
# 替换 YOUR_API_KEY 后执行
cat > ~/.claude/settings.json << 'EOF'
{
  "env": {
    "ANTHROPIC_AUTH_TOKEN": "sk-YOUR_API_KEY",
    "ANTHROPIC_BASE_URL": "https://api.deepseek.com/anthropic",
    "ANTHROPIC_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_OPUS_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_SONNET_MODEL": "deepseek-v4-pro",
    "ANTHROPIC_DEFAULT_HAIKU_MODEL": "deepseek-v4-flash",
    "CLAUDE_CODE_SUBAGENT_MODEL": "deepseek-v4-flash",
    "CLAUDE_CODE_EFFORT_LEVEL": "max"
  },
  "permissions": {
    "allow": [],
    "deny": []
  }
}
EOF
```

---

## 6️⃣ 验证配置

### 检查配置文件

```bash
# 查看配置文件内容
cat ~/.claude/settings.json | python3 -m json.tool

# 预期输出应显示所有配置项
```

### 测试 Claude Code

```bash
# 方式1：启动交互模式
claude

# 方式2：非交互模式测试
echo "你好，请用一句话介绍你自己" | claude -p

# 方式3：代码生成测试
echo "用Python写一个快速排序算法" | claude -p
```

**预期结果**：
- ✅ 成功连接到 DeepSeek API
- ✅ 显示模型信息为 `deepseek-v4-pro`
- ✅ 能正常生成回复

---

## 7️⃣ VSCode 远程 SSH 连接

### 检查 SSH 服务

```bash
# 检查 SSH 是否运行
sudo systemctl status ssh

# 如果未运行，启动服务
sudo systemctl enable ssh
sudo systemctl start ssh

# 获取本机 IP 地址
hostname -I | awk '{print $1}'
```

### 在本地 VSCode 中配置

#### 步骤 1：安装 Remote-SSH 扩展
1. 打开 VSCode
2. 按 `Ctrl+Shift+X` 打开扩展商店
3. 搜索 **"Remote - SSH"**
4. 点击安装（Microsoft 官方发布）

#### 步骤 2：添加 SSH Host
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Remote-SSH: Connect to Host...`
3. 选择 `Add New SSH Host...`
4. 输入连接命令：
   ```
   ssh 用户名@服务器IP地址
   ```
   示例：`ssh xiangcong@192.168.154.128`
5. 选择配置文件保存位置（默认即可）

#### 步骤 3：首次连接
1. 在弹出的列表中选择刚添加的 Host
2. 选择操作系统类型（Linux）
3. 输入密码
4. 连接成功后，左下角显示 `SSH: xxx.xxx.xxx.xxx`

### 配置 SSH 免密登录（推荐）

```bash
# 在本地机器上生成密钥（如果没有）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 复制公钥到服务器
ssh-copy-id 用户名@服务器IP地址

# 测试免密登录
ssh 用户名@服务器IP地址
```

---

## 8️⃣ 常用命令速查

### Claude Code 命令

```bash
# 启动 Claude Code
claude

# 非交互模式（管道输入）
echo "你的问题" | claude -p

# 指定提示继续上一次对话
claude --continue

# 查看版本
claude --version

# 查看帮助
claude --help
```

### Claude Code 交互快捷键

| 快捷键 | 功能 |
|--------|------|
| `Enter` | 发送消息 |
| `Esc` | 取消当前操作 |
| `Esc + Esc` (双击) | 回退对话/代码 |
| `Ctrl+C` | 中断生成 |
| `/help` | 查看帮助 |
| `/model` | 切换模型 |
| `/compact` | 压缩上下文 |

### 环境变量快速设置（临时）

```bash
# 临时切换模型（仅当前终端会话有效）
export ANTHROPIC_MODEL="deepseek-reasoner"

# 启动 Claude Code
claude
```

---

## 9️⃣ 故障排除

### 问题 1：无法连接到 DeepSeek API

**症状**：`Error: Failed to connect to API`

**解决方案**：
```bash
# 检查网络连通性
curl -I https://api.deepseek.com

# 如果需要代理
export HTTP_PROXY="http://127.0.0.1:7890"
export HTTPS_PROXY="http://127.0.0.1:7890"
```

### 问题 2：权限错误 EACCES

**症状**：`npm error Error: EACCES: permission denied`

**解决方案**：
```bash
# 使用 sudo 安装全局包
sudo npm install -g @anthropic-ai/claude-code

# 或者修改 npm 默认目录（不推荐新手）
mkdir ~/.npm-global
npm config set prefix '~/.npm-global'
echo 'export PATH=~/.npm-global/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

### 问题 3：SSH 连接被拒绝

**症状**：`Connection refused`

**解决方案**：
```bash
# 检查 SSH 服务状态
sudo systemctl status ssh

# 启动 SSH 服务
sudo systemctl start ssh

# 设置开机自启
sudo systemctl enable ssh

# 检查防火墙
sudo ufw allow 22
```

### 问题 4：模型响应慢

**原因**：启用了最大思考强度 (`EFFORT_LEVEL=max`)

**解决方案**：
- 这是正常现象，V4-Pro 正在进行深度推理
- 如需更快响应，可将 `CLAUDE_CODE_EFFORT_LEVEL` 改为 `medium` 或 `low`
- 简单任务会自动使用 V4-Flash 模型（较快）

### 问题 5：API Key 无效

**症状**：`Authentication failed` 或 `Invalid API key`

**解决方案**：
1. 检查 `~/.claude/settings.json` 中的 API Key 是否正确
2. 确认 API Key 未过期
3. 检查账户余额是否充足
4. 重新生成 API Key 并更新配置

---

## 📊 性能优化建议

### 1. 模型选择策略

| 场景 | 推荐模型 | 特点 |
|------|---------|------|
| 复杂架构设计 | `deepseek-v4-pro` | 最强推理能力 |
| 大量代码生成 | `deepseek-v4-pro` | 高质量输出 |
| 简单问题回答 | `deepseek-v4-flash` | 快速响应 |
| 代码调试/修复 | `deepseek-v4-pro` | 精准定位问题 |
| 文档生成 | `deepseek-v4-flash` | 效率高 |

### 2. 思考强度调整

```json
// 在 settings.json 中修改 CLAUDE_CODE_EFFORT_LEVEL
"CLAUDE_CODE_EFFORT_LEVEL": "max"      // 最强思考（推荐复杂任务）
"CLAUDE_CODE_EFFORT_LEVEL": "high"     // 高强度思考
"CLAUDE_CODE_EFFORT_LEVEL": "medium"   // 中等强度（平衡速度和质量）
"CLAUDE_CODE_EFFORT_LEVEL": "low"      // 低强度（最快速度）
```

### 3. Token 优化

- 长对话建议定期使用 `/compact` 压缩上下文
- 使用 `@文件名` 引用具体文件而非整个目录
- 避免粘贴过长的日志或代码

---

## 🔄 升级与维护

### 更新 Claude Code

```bash
# 更新到最新版
sudo npm update -g @anthropic-ai/claude-code

# 查看当前版本
claude --version
```

### 切换回官方 Claude 模型（可选）

如果需要临时使用官方 Anthropic 模型：

```bash
# 备份当前配置
cp ~/.claude/settings.json ~/.claude/settings.json.backup

# 编辑配置，注释掉或删除自定义配置
# 或使用环境变量覆盖
unset ANTHROPIC_BASE_URL
unset ANTHROPIC_AUTH_TOKEN
claude  # 将使用官方认证流程
```

---

## 💡 最佳实践

### 1. 项目初始化

```bash
cd /your/project/path
git init
claude  # 在项目根目录启动
```

### 2. 团队协作

将 `.claude/settings.json` 加入 `.gitignore`：
```bash
echo ".claude/" >> .gitignore
```

### 3. 多项目不同配置

可以为不同项目创建不同的配置脚本：

```bash
#!/bin/bash
# project-setup.sh
export ANTHROPIC_MODEL="deepseek-v4-pro"
export CLAUDE_CODE_EFFORT_LEVEL="max"
claude
```

---

## 📞 技术支持

- **DeepSeek 文档**：https://platform.deepseek.com/docs
- **Claude Code 文档**：https://docs.anthropic.com/claude-code
- **GitHub Issues**：https://github.com/anthropics/claude-code/issues

---

## ✅ 配置检查清单

完成所有步骤后，使用此清单验证：

- [ ] Node.js 版本 >= 20.x ✓
- [ ] npm 版本 >= 10.x ✓
- [ ] Claude Code 已安装且可运行 ✓
- [ ] DeepSeek API Key 已配置 ✓
- [ ] 配置文件 JSON 格式正确 ✓
- [ ] SSH 服务正在运行 ✓
- [ ] VSCode Remote-SSH 可正常连接 ✓
- [ ] Claude Code 能成功调用 DeepSeek V4-Pro ✓

---

## 🎉 开始使用！

恭喜！你已经完成了全部配置。现在可以享受 AI 驱动的开发体验：

```bash
# 进入你的项目目录
cd /path/to/your/project

# 启动 Claude Code + DeepSeek V4-Pro
claude

# 开始你的 AI 编程之旅！🚀
```

---

> **文档维护**：如有更新或问题，欢迎反馈改进！  
> **祝开发愉快！** 😊
