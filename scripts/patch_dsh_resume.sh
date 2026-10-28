#!/usr/bin/env bash
# 给 dsh 的 SDK 打上 dsh-bridge 需要的 session/resume 补丁（幂等）。
#
#   bash scripts/patch_dsh_resume.sh            # 打补丁
#   bash scripts/patch_dsh_resume.sh --check    # 只检查是否已打
#
# 背景：官方协议只有 initialize / session/prompt / shutdown，没有 resume；
# dsh-bridge 依赖它把持久化会话从磁盘恢复。升级 dsh 后补丁会丢，需要重打。
set +u

TARGET="$(npm root -g 2>/dev/null)/@deepseek-ai/dsh/node_modules/@deepseek-ai/dsh-sdk-jsonrpc-server/lib/index.js"
if [ ! -f "$TARGET" ]; then
  echo "找不到 $TARGET"; echo "先确认已安装 dsh：npm i -g @deepseek-ai/dsh@<版本>"; exit 1
fi
echo "目标: $TARGET"

if grep -q "session/resume" "$TARGET"; then
  echo "✅ 已打过补丁（检测到 session/resume）"
  exit 0
fi
[ "$1" = "--check" ] && { echo "❌ 未打补丁"; exit 1; }

python3 - "$TARGET" <<'PY'
import re, sys, pathlib
p = pathlib.Path(sys.argv[1]); src = p.read_text()

# 1) 在 case "session/prompt" 前面插入 case（保持同样的缩进）
m = re.search(r'(\n[ \t]*)case\s+"session/prompt"', src)
if not m:
    sys.exit("找不到 case \"session/prompt\" 锚点，版本可能不兼容，请人工处理")
indent = m.group(1)
src = src.replace(m.group(0), f'{indent}case "session/resume": return this.resumeSession(params);' + m.group(0), 1)

# 2) 在 shutdown 方法之前插入 resumeSession 实现
m2 = re.search(r'\n([ \t]*)async shutdown\s*\(', src)
if not m2:
    sys.exit("找不到 async shutdown 锚点，请人工处理")
indent2 = m2.group(1)
method = f'''
{indent2}/** dsh-bridge 补丁：从磁盘恢复已持久化的会话并在本进程继续。 */
{indent2}async resumeSession(params) {{
{indent2}	if (this.shuttingDown) throw new Error("SDK server is shutting down");
{indent2}	const sessionId = params?.sessionId;
{indent2}	if (typeof sessionId !== "string" || sessionId === "") throw new Error("session/resume requires a sessionId string");
{indent2}	if (this.sessions.has(sessionId)) return {{ resumed: true, sessionId }};
{indent2}	const handle = await this.ctx.agents.resume({{ resumeSessionId: sessionId }});
{indent2}	this.sessions.set(sessionId, handle);
{indent2}	return {{ resumed: true, sessionId }};
{indent2}}}
'''
src = src[:m2.start()] + method + src[m2.start():]
p.write_text(src)
print("已写入补丁")
PY

node --check "$TARGET" && echo "✅ 语法检查通过" || { echo "❌ 语法错误，请恢复备份"; exit 1; }
