// 用法: node scripts/ws_import.js x y z yaw  → 导入工件并打印模型 id（workpiece_N）
const WebSocket = require("/home/xz/.local/node/lib/node_modules/@deepseek-ai/dsh/node_modules/ws");
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
