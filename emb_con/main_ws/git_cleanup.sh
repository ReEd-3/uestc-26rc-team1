#!/usr/bin/env bash
# 用法：在仓库可写后，从 main_ws 目录执行：
#   bash git_cleanup.sh
# 该脚本只把生成/IDE/会话文件从 Git 索引移除，不删除本地文件。
set -euo pipefail

cd "$(dirname "$0")/../.."

echo "==> 更新根 .gitignore（若当前挂载可写）"
cat > .gitignore <<'GITIGNORE_EOF'
# Build / 生成目录
build/
Debug/
Release/
MDK-ARM/
.cache/

# IDE / 工具本地配置
.settings/
.vscode/
.dsh/

# 编译中间文件
*.o
*.d
*.elf
*.hex
*.bin
*.map
*.crf
*.lnp
*.axf
*.htm
*.dep
*.build_log.htm
*.uvguix.*
*.uvoptx
*.uvprojx
*.scvd
*.dbgconf

# 临时文件
*.bak
*.tmp
*~
GITIGNORE_EOF

echo "==> 更新 main_ws/.gitignore"
cat > emb_con/main_ws/.gitignore <<'GITIGNORE_EOF'
# Build / 生成目录
build/
Debug/
.cache/

# IDE / 工具本地配置
.settings/
.vscode/
.dsh/

# 临时/测试工程（保留本地，不入库）
repos/go8010_repo/gotest/

# 编译中间文件
*.o
*.d
*.elf
*.hex
*.bin
*.map
*.crf
*.lnp
*.axf
*.htm
*.dep
*.build_log.htm

# 其他
mx.scratch
GITIGNORE_EOF

echo "==> 从索引移除已跟踪的生成/IDE/会话文件（工作区文件保留）"
git ls-files -z \
  | grep -zE '(^|/)(build|Debug|Release|MDK-ARM|\.cache|\.settings|\.vscode|\.dsh)/' \
  | xargs -0 -r git rm --cached --ignore-unmatch --

echo "==> 从索引移除编译中间文件"
git ls-files -z \
  | grep -zE '\.(o|d|elf|hex|bin|map|crf|lnp|axf|htm|dep)$' \
  | xargs -0 -r git rm --cached --ignore-unmatch --

echo "==> 从索引移除 go8010 内嵌测试工程（本地保留）"
git rm -r --cached --ignore-unmatch -- emb_con/main_ws/repos/go8010_repo/gotest || true

echo "==> 暂存 .gitignore"
git add .gitignore emb_con/main_ws/.gitignore || true

echo "==> 完成。请检查："
git status --short | sed -n '1,120p'
echo
echo "确认无误后提交，例如："
echo "  git commit -m \"chore(git): untrack generated and IDE files\""
