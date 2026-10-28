// 用法: node scripts/ws_import.js x y z yaw  → 导入工件并打印模型 id（workpiece_N）
// 便携解析：优先用本机安装的 ws（npm i -g ws 或项目内 npm i ws），
// 都找不到再回退到本机开发环境里的那份（老机器上曾经这样用过）。
function loadWs() {
  const candidates = [
    "ws",
    "/home/xz/.local/node/lib/node_modules/@deepseek-ai/dsh/node_modules/ws",
  ];
  for (const c of candidates) {
    try { return require(c); } catch (e) { /* 下一个 */ }
  }
  throw new Error("缺少 ws 模块：请先运行 npm install -g ws（或在本目录 npm i ws）");
}
const WebSocket = loadWs();
const [x, y, z, yaw] = process.argv.slice(2).map(Number);
const ws = new WebSocket("ws://127.0.0.1:8765/");
let sawSeams = false;
ws.on("open", () => ws.send(JSON.stringify({
  type: "import",
  file_path: "/home/xz/rosim_ws/src/rosim_description/meshes/workpiece_sample.step",
  pose: { position: { x, y, z },
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } },
})));
ws.on("message", (d) => {
  const m = JSON.parse(d.toString());
  if (m.type === "seams") { sawSeams = true; console.log(`seams=${(m.seams || []).length}`); }
  if (m.type === "status" && /imported/.test(m.message || "")) {
    console.log(m.message.replace("imported ", ""));   // -> workpiece_N
    setTimeout(() => process.exit(0), 500);
  }
});
setTimeout(() => { console.log("import-timeout"); process.exit(0); }, 40000);
