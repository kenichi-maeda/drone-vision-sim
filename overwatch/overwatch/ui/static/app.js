// OVERWATCH browser console.
// Each frame packet from the server holds one camera JPEG and the detections
// computed on that same frame. They are drawn together, so the overlay can
// never belong to a different image than the one shown.
"use strict";

const COLORS = { red: "#ff4d4d", orange: "#ff9a3c", yellow: "#ffe14d", green: "#4fd67a",
                 blue: "#4c9bff", purple: "#c35cff", unknown: "#c8cdd2" };
const LOCK = "#ff3b30";
const MOTION = ["forward", "back", "left", "right", "up", "down", "yaw_left", "yaw_right"];
const $ = (id) => document.getElementById(id);

const canvas = $("canvas");
const ctx = canvas.getContext("2d");
const cmdInput = $("cmd");
const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let ws = null;
let keymap = new Map();          // "KeyW" / "Shift+Escape" -> action
let frame = null;                // { bitmap, header }
let status = null;
const held = new Set();

// ------------------------------------------------------------------ socket
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = "arraybuffer";
  ws.onmessage = (event) => (typeof event.data === "string" ? onJson(JSON.parse(event.data)) : onFrame(event.data));
  ws.onclose = () => {
    addLog("error", "Console server disconnected; reconnecting…");
    showBanners([{ level: "critical", text: "CONSOLE OFFLINE", detail: "no connection to the OVERWATCH server" }]);
    setTimeout(connect, 1000);
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

async function onFrame(buffer) {
  const view = new DataView(buffer);
  const headLen = view.getUint32(0);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headLen)));
  const blob = new Blob([new Uint8Array(buffer, 4 + headLen)], { type: "image/jpeg" });
  const bitmap = await createImageBitmap(blob);
  if (frame && frame.bitmap) frame.bitmap.close();
  frame = { bitmap, header };
  if (canvas.width !== header.width || canvas.height !== header.height) {
    canvas.width = header.width;
    canvas.height = header.height;
  }
  $("empty").hidden = true;
  renderDetections(header);
}

function onJson(msg) {
  if (msg.type === "config") {
    keymap = new Map();
    for (const [action, keys] of Object.entries(msg.keys)) for (const key of keys) keymap.set(key, action);
    $("kill-note").textContent = msg.estop_action === "land"
      ? "KILL is configured to land (estop_action: land)."
      : "KILL stops the motors. The drone falls.";
  } else if (msg.type === "status") {
    status = msg;
    renderStatus(msg);
  } else if (msg.type === "log") {
    addLog(msg.level, msg.text);
  } else if (msg.type === "reply") {
    renderOptions(msg.kind === "clarify" ? msg.options : []);
  }
}

// ------------------------------------------------------------------ drawing
function draw(now) {
  requestAnimationFrame(draw);
  if (!frame) return;
  const { bitmap, header } = frame;
  ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
  const target = header.target || {};
  const locked = target.status === "TRACKING" ? target.track_id : null;

  // Image center (where a future pursuit controller steers the target to).
  ctx.strokeStyle = "rgba(255,255,255,0.35)";
  ctx.lineWidth = 1;
  const cx = canvas.width / 2, cy = canvas.height / 2;
  ctx.beginPath(); ctx.moveTo(cx - 8, cy); ctx.lineTo(cx + 8, cy); ctx.moveTo(cx, cy - 8); ctx.lineTo(cx, cy + 8); ctx.stroke();

  for (const d of header.detections) {
    const [x1, y1, x2, y2] = d.box;
    const isLock = d.track_id !== null && d.track_id === locked;
    const isCursor = d.track_id !== null && d.track_id === header.cursor;
    const color = isLock ? LOCK : COLORS[d.color] || COLORS.unknown;
    ctx.lineWidth = isLock ? 3 : 2;
    ctx.setLineDash([]);
    ctx.strokeStyle = color;
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
    if (isCursor && !isLock) {
      ctx.setLineDash([6, 4]);
      ctx.strokeStyle = "#ffffff";
      ctx.strokeRect(x1 - 5, y1 - 5, x2 - x1 + 10, y2 - y1 + 10);
      ctx.setLineDash([]);
    }
    label(`#${d.track_id ?? "?"} ${d.color === "unknown" ? "" : d.color + " "}${d.class_name} ${d.confidence.toFixed(2)}`,
          x1, y1, color);
    if (isLock) reticle(d.box, now, `LOCKED #${d.track_id}`);
  }
  if (target.status === "LOST" && target.bbox_xyxy_px) {
    const [x1, y1, x2, y2] = target.bbox_xyxy_px;
    ctx.setLineDash([8, 6]);
    ctx.strokeStyle = LOCK;
    ctx.lineWidth = 2;
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
    ctx.setLineDash([]);
    label(`LOST #${target.track_id}`, x1, y2 + 18, LOCK);
  }
}

function label(text, x, y, color) {
  ctx.font = "600 13px ui-monospace, Menlo, monospace";
  const w = ctx.measureText(text).width + 8;
  const top = Math.max(0, y - 18);
  ctx.fillStyle = "rgba(0,0,0,0.7)";
  ctx.fillRect(x, top, w, 17);
  ctx.fillStyle = color;
  ctx.fillText(text, x + 4, top + 13);
}

function reticle(box, now, text) {
  const [x1, y1, x2, y2] = box;
  const cx = (x1 + x2) / 2, cy = (y1 + y2) / 2;
  const pulse = reduceMotion ? 0 : 4 * Math.sin(now / 160);
  const r = Math.max(18, Math.max(x2 - x1, y2 - y1) / 2 + 12) + pulse;
  ctx.strokeStyle = LOCK;
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath();
  for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
    ctx.moveTo(cx + dx * (r - 8), cy + dy * (r - 8));
    ctx.lineTo(cx + dx * (r + 12), cy + dy * (r + 12));
  }
  ctx.stroke();
  label(text, cx - 40, cy + r + 30, LOCK);
}

// ------------------------------------------------------------------ panels
function renderDetections(header) {
  const list = $("dets");
  const target = header.target || {};
  $("det-count").textContent = header.detections.length;
  list.replaceChildren(...[...header.detections]
    .sort((a, b) => a.box[0] - b.box[0])
    .map((d) => {
      const li = document.createElement("li");
      if (d.track_id !== null && d.track_id === header.cursor) li.classList.add("cursor");
      if (d.track_id !== null && d.track_id === target.track_id && target.status === "TRACKING") li.classList.add("locked");
      const sw = document.createElement("span");
      sw.className = "sw";
      sw.style.background = COLORS[d.color] || COLORS.unknown;
      const id = document.createElement("span");
      id.className = "num";
      id.textContent = `#${d.track_id ?? "?"}`;
      const what = document.createElement("span");
      what.textContent = `${d.color === "unknown" ? "" : d.color + " "}${d.class_name}`;
      const conf = document.createElement("span");
      conf.className = "conf";
      conf.textContent = d.confidence.toFixed(2);
      li.append(sw, id, what, conf);
      li.title = d.track_id === null ? "Not tracked yet" : "Click to lock";
      li.onclick = () => d.track_id !== null && send({ type: "text", text: `#${d.track_id}` });
      return li;
    }));
}

function renderStatus(s) {
  const c = s.control, v = c.vehicle;
  setChip("chip-source", `SOURCE ${s.source.toUpperCase()}`, null);
  setChip("chip-link", `LINK ${v.link_connected ? "UP" : "DOWN"}`, v.link_connected);
  setChip("chip-px4", `PX4 ${v.autopilot_connected ? "OK" : "—"}`, v.autopilot_connected);
  const manual = c.mode === "MANUAL";
  const mode = $("chip-mode");
  mode.textContent = c.mode;
  mode.classList.toggle("manual", manual);
  const arm = $("chip-arm");
  arm.textContent = v.armed ? "ARMED" : "DISARMED";
  arm.classList.toggle("armed", v.armed);
  $("chip-fps").textContent = `FPS ${s.feed.fps.toFixed(0)} / ${s.display_fps.toFixed(0)}`;
  $("chip-infer").textContent = `INFER ${s.inference_ms.toFixed(0)} ms`;
  const rec = $("chip-rec");
  rec.hidden = !s.recording.active;
  rec.textContent = `REC ${s.recording.frames}`;
  $("empty").hidden = !!frame && !s.health.feed_lost;
  if (s.health.feed_lost) $("empty").textContent = "No camera frames";

  showBanners(s.banners);

  const cmd = c.command;
  const hudCmd = $("hud-cmd");
  hudCmd.textContent = `${c.mode}  fwd ${fmt(cmd.forward)}  left ${fmt(cmd.left)}  up ${fmt(cmd.up)}  yaw ${fmt(cmd.yaw_rate)}`;
  hudCmd.classList.toggle("active", manual && (cmd.forward || cmd.left || cmd.up || cmd.yaw_rate));
  $("hud-block").textContent = manual && c.block_reason && c.block_reason !== "NO OPERATOR INPUT"
    ? `Motion held: ${c.block_reason}` : "";

  const modeBtn = $("btn-mode");
  modeBtn.classList.toggle("manual", manual);
  modeBtn.firstChild.textContent = manual ? "Back to autonomous " : "Manual override ";
  document.querySelector(".flight").classList.toggle("disabled", !manual);
  for (const el of document.querySelectorAll(".keys span")) el.classList.toggle("held", c.held.includes(el.dataset.k));
  $("v-mode").textContent = v.mode || "—";
  $("v-phase").textContent = v.phase || "—";
  $("v-alt").textContent = v.altitude_m === null || v.altitude_m === undefined ? "—" : `${v.altitude_m.toFixed(2)} m`;

  const t = s.target;
  const pill = $("target-status");
  pill.textContent = t.status;
  pill.className = `pill ${t.status}`;
  $("t-id").textContent = t.track_id === null || t.track_id === undefined ? "—" : `#${t.track_id}`;
  $("t-what").textContent = t.class_name ? `${t.color === "unknown" ? "" : t.color + " "}${t.class_name}` : "—";
  $("t-conf").textContent = t.confidence ? t.confidence.toFixed(2) : "—";
  $("t-center").textContent = t.center_norm ? `${t.center_norm[0].toFixed(2)}, ${t.center_norm[1].toFixed(2)}` : "—";
  $("t-error").textContent = t.error_norm ? `${fmt(t.error_norm[0])}, ${fmt(t.error_norm[1])}` : "—";
  $("t-seen").textContent = t.seconds_since_seen === null || t.seconds_since_seen === undefined
    ? "—" : `${t.seconds_since_seen.toFixed(1)} s ago`;
}

const fmt = (n) => (n >= 0 ? "+" : "") + n.toFixed(2);

function setChip(id, text, good) {
  const el = $(id);
  el.textContent = text;
  el.classList.toggle("good", good === true);
  el.classList.toggle("bad", good === false);
}

let lastBanners = "";
function showBanners(list) {
  const key = JSON.stringify(list);
  if (key === lastBanners) return;
  lastBanners = key;
  $("banners").replaceChildren(...list.map((b) => {
    const div = document.createElement("div");
    div.className = `banner ${b.level}`;
    div.textContent = b.text;
    if (b.detail) { const small = document.createElement("small"); small.textContent = b.detail; div.append(small); }
    return div;
  }));
}

function renderOptions(options) {
  $("options").replaceChildren(...(options || []).map((o) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "btn";
    b.textContent = `${o.n}) #${o.track_id} ${o.label}`;
    b.onclick = () => send({ type: "text", text: String(o.n) });
    return b;
  }));
}

function addLog(level, text) {
  const li = document.createElement("li");
  li.className = text.startsWith("> ") ? "cmd" : level;
  li.textContent = text;
  const log = $("log");
  log.prepend(li);
  while (log.children.length > 80) log.lastChild.remove();
}

// ------------------------------------------------------------------ input
function keyName(e) {
  return e.shiftKey && !e.code.startsWith("Shift") ? `Shift+${e.code}` : e.code;
}

function typing() { return document.activeElement === cmdInput; }

window.addEventListener("keydown", (e) => {
  const action = keymap.get(keyName(e)) || (e.shiftKey ? null : keymap.get(e.code));
  // KILL works everywhere, including while typing a command.
  if (action === "kill") { e.preventDefault(); if (!e.repeat) send({ type: "action", action: "kill" }); return; }
  if (typing()) {
    if (e.code === "Escape") { cmdInput.blur(); canvas.focus(); }
    return;
  }
  if (!action) return;
  e.preventDefault();
  if (MOTION.includes(action)) { held.add(action); pushInput(); return; }
  if (e.repeat) return;
  switch (action) {
    case "cycle_next": send({ type: "cycle", direction: 1 }); break;
    case "cycle_prev": send({ type: "cycle", direction: -1 }); break;
    case "lock": send({ type: "lock" }); break;
    case "release": send({ type: "release" }); break;
    case "focus_command": cmdInput.focus(); break;
    default: send({ type: "action", action });
  }
}, true);

window.addEventListener("keyup", (e) => {
  for (const key of [keyName(e), e.code]) {
    const action = keymap.get(key);
    if (action && MOTION.includes(action)) held.delete(action);
  }
  pushInput();
}, true);

function releaseAll() { held.clear(); pushInput(); }
window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => document.hidden && releaseAll());
cmdInput.addEventListener("focus", releaseAll);

function pushInput() { send({ type: "input", keys: [...held], typing: typing() }); }
setInterval(pushInput, 50);  // the server treats input older than 250 ms as "no input"

canvas.addEventListener("click", (e) => {
  if (!frame) return;
  const rect = canvas.getBoundingClientRect();
  const x = (e.clientX - rect.left) * (canvas.width / rect.width);
  const y = (e.clientY - rect.top) * (canvas.height / rect.height);
  send({ type: "click", seq: frame.header.seq, x, y });  // judged against the frame that was on screen
  canvas.focus();
});

$("cmd-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const text = cmdInput.value.trim();
  if (text) send({ type: "text", text });
  cmdInput.value = "";
});
$("btn-release").onclick = () => send({ type: "release" });
$("btn-mode").onclick = () => send({ type: "action", action: "toggle_mode", source: "button" });
for (const b of document.querySelectorAll("[data-action]")) {
  b.addEventListener("click", () => send({ type: "action", action: b.dataset.action, source: "button" }));
}

connect();
requestAnimationFrame(draw);
canvas.focus();
