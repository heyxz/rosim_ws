#!/usr/bin/env bash
# 给 dsh 的 SDK 打上 dsh-bridge 需要的 session/resume 补丁（幂等）。
#
#   bash scripts/patch_dsh_resume.sh            # 打补丁
#   bash scripts/patch_dsh_resume.sh --check    # 只检查是否已打
#
# 背景：官方协议只有 initialize / session/prompt / shutdown，没有 resume；
# dsh-bridge 依赖它把持久化会话从磁盘恢复。升级 dsh 后补丁会丢，需要重打。
set +u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# dsh-bridge 自带一个补丁脚本，锚点同时兼容 0.2.0 / 0.2.1（推荐用它）
BRIDGE_PATCH="$ROOT/dsh-bridge/tools/patch-sdk-resume.mjs"
if [ -f "$BRIDGE_PATCH" ] && command -v node >/dev/null; then
  echo "使用 dsh-bridge 自带补丁脚本：$BRIDGE_PATCH"
  exec node "$BRIDGE_PATCH" "$@"
fi

TARGET="${DSH_SDK_FILE:-$(npm root -g 2>/dev/null)/@deepseek-ai/dsh/node_modules/@deepseek-ai/dsh-sdk-jsonrpc-server/lib/index.js}"
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

# 2) 找方法插入点：优先 hasAdapterFor（0.2.1+），其次 shutdown（0.2.0）
m2 = None
for pat in (r'\n([ \t]*)hasAdapterFor\s*\(', r'\n([ \t]*)async shutdown\s*\(', r'\n([ \t]*)shutdown\s*\('):
    m2 = re.search(pat, src)
    if m2:
        break
if not m2:
    sys.exit("""找不到插入锚点（hasAdapterFor / shutdown 都不在）。
两种处理：
  1) 先把 dsh-bridge 解包到 ~/rosim_ws/dsh-bridge（它有自带补丁脚本，兼容性更好）
  2) 或直接跳过补丁：bridge 恢复会话失败时会自动改用新会话，
     上下文仍由 docs/memory.md 保证（只是飞书聊天会开新会话）""")
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
