// arctos-pi minimal jog UI. Vanilla JS, no build step.

const state = {
  axes: [], cfg: null, speed: 0.25, ws: null, lastPong: 0,
  gripperHoldUntil: 0,
  controlMode: "manual",   // "manual" (jog) | "programs" (ROS trajectories)
  rosReady: false,         // ROS bridge action server reachable
  allHomed: false,         // every home_enabled axis is homed
  teach: { count: 0, loaded_name: null, dirty: false, waypoints: [] },
};
const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

async function api(path, opts = {}) {
  const res = await fetch(path, {
    method: opts.method || "GET",
    headers: { "Content-Type": "application/json" },
    body: opts.body ? JSON.stringify(opts.body) : undefined,
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || "request failed");
  }
  return res.json();
}

function setStatus(msg, cls = "text-gray-300") {
  const s = $("#status");
  s.className = "flex-1 text-sm " + cls;
  s.textContent = msg;
}

function renderJogAxes() {
  const host = $("#jog-axes");
  host.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const row = document.createElement("div");
    row.className = "flex items-center gap-2 bg-gray-900 rounded-lg p-2";
    row.innerHTML = `
      <div class="w-14 font-bold">${ax.name}</div>
      <button class="btn jogbtn bg-gray-700 rounded-lg flex-1" data-can="${ax.can_id}" data-dir="0">−</button>
      <div class="pos w-24 text-right text-lg font-mono" id="pos-${ax.can_id}">0.00°</div>
      <button class="btn jogbtn bg-gray-700 rounded-lg flex-1" data-can="${ax.can_id}" data-dir="1">+</button>
      <input class="goinput w-20 bg-gray-800 rounded-lg px-2 py-2 text-sm text-right font-mono"
             type="number" inputmode="decimal" step="1"
             min="${ax.soft_limit_min}" max="${ax.soft_limit_max}"
             data-can="${ax.can_id}" placeholder="angle°" aria-label="${ax.name} target angle" />
      <button class="btn gobtn bg-blue-700 hover:bg-blue-600 rounded-lg px-3 py-2 text-sm font-semibold"
              data-can="${ax.can_id}">Go</button>
    `;
    host.appendChild(row);
  }
  // Bind the per-joint "Go to angle" inputs: type a target and press Go (or Enter).
  // Uses the same /api/move endpoint as everything else, which routes to the ROS
  // bridge when in ROS mode. The differential wrist coordinates J5/J6 server-side.
  for (const b of $$(".gobtn")) {
    const can = parseInt(b.dataset.can, 10);
    const input = $(`.goinput[data-can="${can}"]`);
    const go = () => moveToAngle(can, parseFloat(input.value));
    b.addEventListener("click", go);
    input.addEventListener("keydown", (e) => {
      if (e.key === "Enter") { e.preventDefault(); go(); }
    });
  }
  // Bind hold-to-jog. While held we REPUBLISH at ~8 Hz: in ROS mode the bridge
  // runs a deadman that stops the axis if it stops hearing fresh jog commands,
  // so a single start would self-cancel after ~0.35 s. Harmless in direct mode.
  const JOG_REPEAT_MS = 120;
  for (const b of $$(".jogbtn")) {
    let timer = null;
    const send = () => {
      const can = parseInt(b.dataset.can, 10);
      const dir = parseInt(b.dataset.dir, 10);
      api("/api/jog/start", { method: "POST", body: { can_id: can, direction: dir, speed_pct: state.speed } })
        .catch((err) => setStatus("jog err: " + err.message, "text-red-400"));
    };
    const start = (e) => {
      if (state.controlMode !== "manual") return;  // Programs mode owns the bus
      e.preventDefault();
      if (timer) return;
      b.classList.add("active");
      send();
      timer = setInterval(send, JOG_REPEAT_MS);
    };
    const stop = (e) => {
      if (!timer) return;
      if (e) e.preventDefault();
      clearInterval(timer);
      timer = null;
      b.classList.remove("active");
      const can = parseInt(b.dataset.can, 10);
      api("/api/jog/stop", { method: "POST", body: { can_id: can } }).catch(() => {});
    };
    b.addEventListener("pointerdown", start);
    b.addEventListener("pointerup", stop);
    b.addEventListener("pointerleave", stop);
    b.addEventListener("pointercancel", stop);
  }
}

async function moveToAngle(can, deg) {
  if (state.controlMode !== "manual") return;  // Programs mode owns the bus
  const ax = state.cfg.axes.find((a) => a.can_id === can);
  const name = ax ? ax.name : `id ${can}`;
  if (!Number.isFinite(deg)) {
    setStatus(`${name}: enter a target angle first`, "text-red-400");
    return;
  }
  // Soft-limit guard mirrors the server (which also rejects); fail fast with a
  // clear message instead of a 400 round-trip.
  if (ax && (deg < ax.soft_limit_min || deg > ax.soft_limit_max)) {
    setStatus(`${name}: ${deg}° outside [${ax.soft_limit_min}, ${ax.soft_limit_max}]`, "text-red-400");
    return;
  }
  try {
    const r = await api("/api/move", {
      method: "POST",
      body: { can_id: can, degrees: deg, speed_pct: state.speed },
    });
    if (r && r.accepted === false) {
      setStatus(`${name}: move rejected${r.error_string ? " — " + r.error_string : ""}`, "text-yellow-400");
    } else {
      setStatus(`${name} → ${deg}°`, "text-blue-300");
    }
  } catch (e) {
    setStatus(`${name} move: ${e.message}`, "text-red-400");
  }
}

function homedCell(s) {
  // Returns {label, cls, title} for the per-axis homed indicator.
  if (s.home_enabled === false) return { label: "n/a", cls: "text-gray-600", title: "no home sensor" };
  if (s.homing) return { label: "homing…", cls: "text-amber-400", title: "seeking home switch" };
  if (s.is_homed) return { label: "✓", cls: "text-emerald-400", title: "homed" };
  return { label: "not homed", cls: "text-red-400", title: s.home_error || "home before absolute moves" };
}

function renderStatus(axes, gripper) {
  const body = $("#status-body");
  body.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const s = axes[ax.name] || {};
    const h = homedCell(s);
    const canHome = s.home_enabled !== false;
    const tr = document.createElement("tr");
    tr.className = "border-t border-gray-800";
    tr.innerHTML = `
      <td class="py-1">${ax.name} <span class="text-gray-500 text-xs">id ${ax.can_id}</span></td>
      <td class="text-right pos">${(s.degrees ?? 0).toFixed(2)}°</td>
      <td class="text-right pos text-gray-400">${s.pulses ?? 0}</td>
      <td class="text-right">${s.enabled ? "✓" : "—"}</td>
      <td class="text-right ${h.cls}" title="${h.title}">${h.label}</td>
      <td class="text-right">
        <button class="home-btn btn bg-amber-700 hover:bg-amber-600 disabled:opacity-40 rounded px-2 py-0.5 text-xs"
                data-can="${ax.can_id}" ${(!canHome || s.homing) ? "disabled" : ""}>Home</button>
      </td>
    `;
    body.appendChild(tr);
    const posEl = document.getElementById(`pos-${ax.can_id}`);
    if (posEl) posEl.textContent = (s.degrees ?? 0).toFixed(2) + "°";
  }
  if (gripper && gripper.present) {
    const tr = document.createElement("tr");
    tr.className = "border-t border-gray-800";
    tr.innerHTML = `
      <td class="py-1">Gripper <span class="text-gray-500 text-xs">id ${gripper.can_id}</span></td>
      <td class="text-right pos text-gray-600">—</td>
      <td class="text-right pos text-gray-400">${gripper.position ?? 0}</td>
      <td class="text-right">${gripper.enabled ? "✓" : "—"}</td>
      <td class="text-right text-gray-600">—</td>
      <td></td>
    `;
    body.appendChild(tr);
  }
  for (const b of $$(".home-btn")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      try {
        await api("/api/home", { method: "POST", body: { can_id: can } });
        setStatus(`homing axis ${can}…`, "text-amber-400");
      } catch (e) { setStatus("home: " + e.message, "text-red-400"); }
    });
  }
}

// ---- Homing validation tab ----
// Built once (stable buttons/handlers); the live bits update each WS tick.
function initHoming() {
  const host = $("#homing-body");
  if (!host) return;
  host.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const row = document.createElement("div");
    row.className = "bg-gray-900 rounded-lg p-3 flex flex-wrap items-center gap-3";
    row.innerHTML = `
      <div class="w-14 font-bold">${ax.name}</div>
      <div class="flex items-center gap-1">
        <span class="text-xs text-gray-400">seek</span>
        <button data-can="${ax.can_id}" data-ccw="0" class="dir-btn btn bg-gray-700 rounded px-2 py-1 text-xs">CW</button>
        <button data-can="${ax.can_id}" data-ccw="1" class="dir-btn btn bg-gray-700 rounded px-2 py-1 text-xs">CCW</button>
      </div>
      <div class="flex items-center gap-1">
        <span class="text-xs text-gray-400">sensor</span>
        <span id="home-sw-${ax.can_id}" class="text-xs font-mono text-gray-600 w-16">—</span>
      </div>
      <div class="flex items-center gap-1">
        <span class="text-xs text-gray-400">status</span>
        <span id="home-st-${ax.can_id}" class="text-xs text-gray-500">—</span>
      </div>
      <button data-can="${ax.can_id}" class="home1-btn ml-auto btn bg-amber-700 hover:bg-amber-600 disabled:opacity-40 rounded px-4 py-1.5 text-sm font-semibold">Home</button>
    `;
    host.appendChild(row);
  }
  for (const b of $$(".dir-btn")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      const ccw = b.dataset.ccw === "1";
      try {
        await api("/api/home/dir", { method: "POST", body: { can_id: can, ccw } });
        setStatus(`axis ${can}: seek ${ccw ? "CCW" : "CW"}`, "text-emerald-400");
      } catch (e) { setStatus("dir: " + e.message, "text-red-400"); }
    });
  }
  for (const b of $$(".home1-btn")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      try {
        await api("/api/home", { method: "POST", body: { can_id: can } });
        setStatus(`homing axis ${can}…`, "text-amber-400");
      } catch (e) { setStatus("home: " + e.message, "text-red-400"); }
    });
  }
}

function renderHomingLive(axes) {
  if (!document.getElementById("home-sw-" + (state.cfg.axes[0]?.can_id))) return;
  for (const ax of state.cfg.axes) {
    const s = axes[ax.name] || {};
    const sw = document.getElementById(`home-sw-${ax.can_id}`);
    if (sw) {
      if (s.home_switch === true) { sw.textContent = "TRIPPED"; sw.className = "text-xs font-mono text-emerald-400 w-16"; }
      else if (s.home_switch === false) { sw.textContent = "clear"; sw.className = "text-xs font-mono text-gray-400 w-16"; }
      else { sw.textContent = "—"; sw.className = "text-xs font-mono text-gray-600 w-16"; }
    }
    const stt = document.getElementById(`home-st-${ax.can_id}`);
    if (stt) { const h = homedCell(s); stt.textContent = h.label; stt.className = "text-xs " + h.cls; stt.title = h.title; }
    const dir = s.home_dir ?? 0;
    for (const b of $$(`.dir-btn[data-can="${ax.can_id}"]`)) {
      const active = parseInt(b.dataset.ccw, 10) === dir;
      b.classList.toggle("bg-blue-600", active);
      b.classList.toggle("bg-gray-700", !active);
    }
    const hb = document.querySelector(`.home1-btn[data-can="${ax.can_id}"]`);
    if (hb) hb.disabled = (s.home_enabled === false) || !!s.homing;
  }
}

function renderGripper(g) {
  if (!g || !g.present) return;
  const slider = $("#gripper-slider");
  $("#gripper-open").disabled = !g.enabled;
  $("#gripper-close").disabled = !g.enabled;
  slider.disabled = !g.enabled;
  // Suppress slider sync briefly after any user interaction so a WS tick
  // landing between pointerup and the backend processing our final POST
  // can't snap the thumb back to a stale value.
  if (Date.now() >= state.gripperHoldUntil) {
    slider.value = String(g.position ?? 0);
    $("#gripper-pos").textContent = String(g.position ?? 0);
  }
}

function renderTeachSummary(t) {
  if (!t) return;
  // Only refetch the full waypoint list if something changed.
  const changed = t.count !== state.teach.count || t.loaded_name !== state.teach.loaded_name || t.dirty !== state.teach.dirty;
  state.teach.count = t.count;
  state.teach.loaded_name = t.loaded_name;
  state.teach.dirty = t.dirty;
  renderTeachLoadedLabel();
  if (changed) refreshTeach().catch(() => {});
}

function renderTeachLoadedLabel() {
  const lbl = $("#teach-loaded");
  if (!lbl) return;
  const name = state.teach.loaded_name;
  const dirty = state.teach.dirty ? " *" : "";
  lbl.textContent = name ? `loaded: ${name}${dirty}` : (state.teach.count ? "— unsaved —" : "— no program loaded —");
}

function renderTeachList() {
  const host = $("#teach-list");
  if (!host) return;
  host.innerHTML = "";
  const wps = state.teach.waypoints || [];
  if (wps.length === 0) {
    host.innerHTML = '<div class="text-xs text-gray-500 italic">No waypoints. Jog to a pose, then "+ Capture waypoint".</div>';
    return;
  }
  wps.forEach((wp, idx) => {
    const row = document.createElement("div");
    row.className = "bg-gray-900 rounded p-2 flex items-center gap-2";
    const joints = Object.entries(wp.joints).map(([k, v]) => `${k}:${Number(v).toFixed(1)}°`).join(" ");
    const grip = (wp.gripper !== undefined && wp.gripper !== null) ? ` <span class="text-amber-400">G${wp.gripper}</span>` : "";
    row.innerHTML = `
      <div class="w-6 text-xs text-gray-500">${idx + 1}</div>
      <div class="flex-1 font-mono text-xs truncate" title="${joints}">${joints}${grip}</div>
      <label class="text-xs text-gray-400">dwell
        <input type="number" min="0" max="600000" step="100" value="${wp.dwell_ms}" data-idx="${idx}"
               class="wp-dwell bg-gray-800 rounded px-1 py-0.5 w-16 ml-1" />
      </label>
      <label class="text-xs text-gray-400">spd
        <input type="number" min="1" max="100" step="1" value="${Math.round(wp.speed_pct * 100)}" data-idx="${idx}"
               class="wp-speed bg-gray-800 rounded px-1 py-0.5 w-12 ml-1" />
      </label>
      <button data-idx="${idx}" data-act="up" class="wp-act btn bg-gray-700 hover:bg-gray-600 rounded px-2 py-0.5 text-xs" ${idx === 0 ? "disabled" : ""}>↑</button>
      <button data-idx="${idx}" data-act="down" class="wp-act btn bg-gray-700 hover:bg-gray-600 rounded px-2 py-0.5 text-xs" ${idx === wps.length - 1 ? "disabled" : ""}>↓</button>
      <button data-idx="${idx}" data-act="del" class="wp-act btn bg-red-700 hover:bg-red-600 rounded px-2 py-0.5 text-xs">×</button>
    `;
    host.appendChild(row);
  });
  for (const inp of $$(".wp-dwell")) {
    inp.addEventListener("change", async () => {
      const idx = parseInt(inp.dataset.idx, 10);
      try { await api(`/api/teach/${idx}`, { method: "PATCH", body: { dwell_ms: parseInt(inp.value, 10) } }); }
      catch (e) { setStatus("teach: " + e.message, "text-red-400"); }
    });
  }
  for (const inp of $$(".wp-speed")) {
    inp.addEventListener("change", async () => {
      const idx = parseInt(inp.dataset.idx, 10);
      try { await api(`/api/teach/${idx}`, { method: "PATCH", body: { speed_pct: parseInt(inp.value, 10) / 100 } }); }
      catch (e) { setStatus("teach: " + e.message, "text-red-400"); }
    });
  }
  for (const b of $$(".wp-act")) {
    b.addEventListener("click", async () => {
      const idx = parseInt(b.dataset.idx, 10);
      try {
        if (b.dataset.act === "del") {
          await api(`/api/teach/${idx}`, { method: "DELETE" });
        } else if (b.dataset.act === "up") {
          await api(`/api/teach/${idx}/reorder`, { method: "POST", body: { to: idx - 1 } });
        } else if (b.dataset.act === "down") {
          await api(`/api/teach/${idx}/reorder`, { method: "POST", body: { to: idx + 1 } });
        }
        await refreshTeach();
      } catch (e) { setStatus("teach: " + e.message, "text-red-400"); }
    });
  }
}

async function refreshTeach() {
  const t = await api("/api/teach");
  state.teach = { count: t.count, loaded_name: t.loaded_name, dirty: t.dirty, waypoints: t.waypoints || [] };
  renderTeachLoadedLabel();
  renderTeachList();
}

async function refreshTeachPrograms() {
  const sel = $("#teach-programs");
  if (!sel) return;
  try {
    const { programs } = await api("/api/teach/programs");
    const prev = sel.value;
    sel.innerHTML = programs.length === 0
      ? '<option value="">— none saved —</option>'
      : programs.map(n => `<option value="${n}">${n}</option>`).join("");
    if (programs.includes(prev)) sel.value = prev;
  } catch (e) { /* non-fatal */ }
}

function initTeach() {
  $("#teach-capture").addEventListener("click", async () => {
    try {
      await api("/api/teach/capture", { method: "POST", body: {
        dwell_ms: parseInt($("#teach-dwell").value, 10) || 0,
        speed_pct: (parseInt($("#teach-speed").value, 10) || 50) / 100,
      }});
      await refreshTeach();
    } catch (e) { setStatus("capture: " + e.message, "text-red-400"); }
  });
  $("#teach-clear").addEventListener("click", async () => {
    if (state.teach.count > 0 && !confirm("Clear all captured waypoints?")) return;
    try { await api("/api/teach/clear", { method: "POST" }); await refreshTeach(); }
    catch (e) { setStatus("teach: " + e.message, "text-red-400"); }
  });
  $("#teach-save").addEventListener("click", async () => {
    const name = $("#teach-name").value.trim() || state.teach.loaded_name;
    if (!name) { setStatus("teach: enter a program name", "text-yellow-400"); return; }
    try {
      await api("/api/teach/save", { method: "POST", body: { name } });
      await refreshTeachPrograms();
      await refreshTeach();
      setStatus(`saved program: ${name}`, "text-emerald-400");
    } catch (e) { setStatus("save: " + e.message, "text-red-400"); }
  });
  $("#teach-load").addEventListener("click", async () => {
    const sel = $("#teach-programs");
    const name = sel.value;
    if (!name) return;
    if (state.teach.dirty && !confirm("Replace unsaved waypoints?")) return;
    try {
      await api("/api/teach/load", { method: "POST", body: { name } });
      $("#teach-name").value = name;
      await refreshTeach();
      setStatus(`loaded program: ${name}`, "text-emerald-400");
    } catch (e) { setStatus("load: " + e.message, "text-red-400"); }
  });
  $("#teach-delete-prog").addEventListener("click", async () => {
    const sel = $("#teach-programs");
    const name = sel.value;
    if (!name) return;
    if (!confirm(`Delete saved program "${name}"?`)) return;
    try {
      await api(`/api/teach/programs/${encodeURIComponent(name)}`, { method: "DELETE" });
      await refreshTeachPrograms();
      setStatus(`deleted program: ${name}`, "text-emerald-400");
    } catch (e) { setStatus("delete: " + e.message, "text-red-400"); }
  });
  refreshTeachPrograms();
  refreshTeach();
}

function initGripper() {
  const g = state.cfg.gripper;
  if (!g || !g.enabled) return;
  $("#gripper-panel").classList.remove("hidden");
  const slider = $("#gripper-slider");
  slider.value = String(g.default_position ?? 0);
  $("#gripper-pos").textContent = String(g.default_position ?? 0);

  let gripperLastSend = 0;
  let gripperPending = null;
  const HOLD_MS = 600; // suppress WS slider-sync this long after any touch
  const holdSlider = () => { state.gripperHoldUntil = Date.now() + HOLD_MS; };
  const sendGripper = async (v) => {
    try {
      await api("/api/gripper", { method: "POST", body: { position: v } });
    } catch (e) {
      setStatus("gripper err: " + e.message, "text-red-400");
    }
  };
  const scheduleGripper = (v) => {
    const now = Date.now();
    const since = now - gripperLastSend;
    if (since >= 50) {
      gripperLastSend = now;
      if (gripperPending) { clearTimeout(gripperPending); gripperPending = null; }
      sendGripper(v);
    } else {
      if (gripperPending) clearTimeout(gripperPending);
      gripperPending = setTimeout(() => {
        gripperPending = null;
        gripperLastSend = Date.now();
        sendGripper(parseInt(slider.value, 10));
      }, 50 - since);
    }
  };

  slider.addEventListener("pointerdown", holdSlider);
  const endDrag = () => {
    holdSlider();
    if (gripperPending) { clearTimeout(gripperPending); gripperPending = null; }
    gripperLastSend = Date.now();
    sendGripper(parseInt(slider.value, 10));
  };
  slider.addEventListener("pointerup", endDrag);
  slider.addEventListener("pointercancel", endDrag);
  slider.addEventListener("input", () => {
    const v = parseInt(slider.value, 10);
    $("#gripper-pos").textContent = String(v);
    holdSlider();
    scheduleGripper(v);
  });
  slider.addEventListener("change", () => {
    holdSlider();
    sendGripper(parseInt(slider.value, 10));
  });
  $("#gripper-open").addEventListener("click", async () => {
    try { await api("/api/gripper/open", { method: "POST" }); }
    catch (e) { setStatus("gripper err: " + e.message, "text-red-400"); }
  });
  $("#gripper-close").addEventListener("click", async () => {
    try { await api("/api/gripper/close", { method: "POST" }); }
    catch (e) { setStatus("gripper err: " + e.message, "text-red-400"); }
  });
}

const WORK_MODES = [
  { v: 5, label: "SR_vFOC (recommended)" },
  { v: 4, label: "SR_CLOSE" },
  { v: 3, label: "SR_OPEN" },
  { v: 2, label: "CR_vFOC" },
  { v: 1, label: "CR_CLOSE" },
  { v: 0, label: "CR_OPEN" },
];

function renderConfig() {
  const host = $("#cfg-body");
  host.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const card = document.createElement("div");
    card.className = "bg-gray-900 rounded-lg p-3 flex flex-wrap items-center gap-3";
    card.innerHTML = `
      <div class="w-14 font-bold">${ax.name}</div>
      <div class="flex items-center gap-2">
        <label class="text-xs text-gray-400">microsteps</label>
        <select data-can="${ax.can_id}" class="ms-sel bg-gray-800 rounded px-2 py-1">
          ${[1, 2, 4, 8, 16, 32, 64, 128, 256].map(v =>
            `<option value="${v}" ${v === ax.default_microsteps ? "selected" : ""}>${v}</option>`).join("")}
        </select>
      </div>
      <div class="flex items-center gap-2">
        <label class="text-xs text-gray-400">current (mA)</label>
        <input type="number" data-can="${ax.can_id}" value="${ax.default_current_ma}" step="100" min="0" max="5200"
               class="cur-in bg-gray-800 rounded px-2 py-1 w-24" />
      </div>
      <button data-can="${ax.can_id}" class="apply-cfg ml-auto bg-amber-600 hover:bg-amber-500 rounded px-3 py-1 text-sm font-semibold">Apply</button>
      <div class="basis-full flex items-center gap-2 pt-2 border-t border-gray-800">
        <label class="text-xs text-gray-400 w-14">gear ratio</label>
        <input type="number" data-can="${ax.can_id}" value="${ax.gear_ratio}" step="0.1" min="0.01"
               class="gear-in bg-gray-800 rounded px-2 py-1 w-28" />
        <span class="text-xs text-gray-500">MoveIt↔motor scaling · software only, not saved — copy into config.yaml once it matches</span>
        <button data-can="${ax.can_id}" class="apply-gear ml-auto bg-blue-600 hover:bg-blue-500 rounded px-3 py-1 text-sm font-semibold">Apply gear ratio</button>
      </div>
      <div class="basis-full flex items-center gap-2 pt-2 border-t border-gray-800">
        <label class="text-xs text-gray-400 w-14">joint zero</label>
        <input type="number" data-can="${ax.can_id}" value="0" step="1"
               class="zero-in bg-gray-800 rounded px-2 py-1 w-28" />
        <span class="text-xs text-gray-500">jog to a known pose, type the TRUE joint angle, calibrate · offset <span class="zero-off" data-can="${ax.can_id}">${(ax.home_offset_deg ?? 0).toFixed(1)}</span>° — copy home_offset_deg into config.yaml</span>
        <button data-can="${ax.can_id}" class="apply-zero ml-auto bg-blue-600 hover:bg-blue-500 rounded px-3 py-1 text-sm font-semibold">Calibrate zero</button>
      </div>
      <div class="basis-full flex items-center gap-2 pt-2 border-t border-gray-800">
        <label class="text-xs text-gray-400 w-14">work mode</label>
        <select data-can="${ax.can_id}" class="wm-sel bg-gray-800 rounded px-2 py-1">
          ${WORK_MODES.map(m => `<option value="${m.v}">${m.label}</option>`).join("")}
        </select>
        <span class="text-xs text-gray-500">writes driver flash · jog needs SR_*</span>
        <button data-can="${ax.can_id}" class="apply-wm ml-auto bg-red-700 hover:bg-red-600 rounded px-3 py-1 text-sm font-semibold">Set work mode</button>
      </div>
    `;
    host.appendChild(card);
  }
  for (const b of $$(".apply-cfg")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      const ms = parseInt(document.querySelector(`.ms-sel[data-can="${can}"]`).value, 10);
      const cur = parseInt(document.querySelector(`.cur-in[data-can="${can}"]`).value, 10);
      try {
        await api("/api/microsteps", { method: "POST", body: { can_id: can, microsteps: ms } });
        await api("/api/current", { method: "POST", body: { can_id: can, milliamps: cur } });
        setStatus(`axis ${can}: microsteps=${ms}, current=${cur}mA applied`, "text-emerald-400");
      } catch (e) {
        setStatus("apply failed: " + e.message, "text-red-400");
      }
    });
  }
  for (const b of $$(".apply-gear")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      const gr = parseFloat(document.querySelector(`.gear-in[data-can="${can}"]`).value);
      if (!(gr > 0)) { setStatus("gear ratio must be > 0", "text-red-400"); return; }
      try {
        await api("/api/gear_ratio", { method: "POST", body: { can_id: can, gear_ratio: gr } });
        const ax = state.cfg.axes.find(a => a.can_id === can);
        if (ax) ax.gear_ratio = gr;
        setStatus(`axis ${can}: gear ratio → ${gr} · test a move, then copy to config.yaml`, "text-emerald-400");
      } catch (e) {
        setStatus("gear ratio failed: " + e.message, "text-red-400");
      }
    });
  }
  // Joint-zero calibration: the displayed angle becomes the entered TRUE joint
  // angle at the current pose (server recomputes home_offset_deg).
  for (const b of $$(".apply-zero")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      const deg = parseFloat(document.querySelector(`.zero-in[data-can="${can}"]`).value);
      if (!Number.isFinite(deg)) { setStatus("enter the joint's true angle first", "text-red-400"); return; }
      try {
        const r = await api("/api/joint_zero", { method: "POST", body: { can_id: can, angle_deg: deg } });
        const ax = state.cfg.axes.find(a => a.can_id === can);
        if (ax) ax.home_offset_deg = r.home_offset_deg;
        const off = document.querySelector(`.zero-off[data-can="${can}"]`);
        if (off) off.textContent = r.home_offset_deg.toFixed(1);
        setStatus(`axis ${can}: now reads ${deg}° here · copy home_offset_deg: ${r.home_offset_deg} into config.yaml`, "text-emerald-400");
      } catch (e) {
        setStatus("joint zero failed: " + e.message, "text-red-400");
      }
    });
  }
  for (const b of $$(".apply-wm")) {
    b.addEventListener("click", async () => {
      const can = parseInt(b.dataset.can, 10);
      const sel = document.querySelector(`.wm-sel[data-can="${can}"]`);
      const mode = parseInt(sel.value, 10);
      const label = sel.options[sel.selectedIndex].text;
      if (!confirm(`Set axis ${can} work mode to ${label}?\n\nThis writes to the driver's flash and persists across power cycles.`)) return;
      try {
        await api("/api/work_mode", { method: "POST", body: { can_id: can, mode } });
        setStatus(`axis ${can}: work mode → ${label}`, "text-emerald-400");
      } catch (e) {
        setStatus("work mode failed: " + e.message, "text-red-400");
      }
    });
  }
}

function connectWs() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${proto}//${location.host}/api/ws`);
  state.ws = ws;
  ws.onopen = () => {
    setStatus(`connected (${state.cfg?.backend || "?"})`, "text-emerald-400");
    setInterval(() => ws.readyState === 1 && ws.send(JSON.stringify({ type: "ping" })), 200);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "state") {
      state.axesLive = msg.axes;
      state.allHomed = Object.values(msg.axes).every(a => a.home_enabled === false || a.is_homed);
      renderStatus(msg.axes, msg.gripper);
      renderHomingLive(msg.axes);
      renderGripper(msg.gripper);
      renderTeachSummary(msg.teach);
      updateProgramButtonsEnabled();
    }
    if (msg.type === "pong") state.lastPong = msg.t;
  };
  ws.onclose = () => {
    setStatus("disconnected, retrying…", "text-yellow-400");
    setTimeout(connectWs, 1000);
  };
}

function moveitLog(msg, cls = "text-gray-400") {
  const el = $("#moveit-log");
  el.className = "text-xs font-mono whitespace-pre-wrap " + cls;
  el.textContent = msg;
}

// Manual-jog vs ROS-programs mode. Only one may drive the bus at a time, so the
// jog pad and the program/MoveIt actions are mutually exclusive in the UI (the
// bridge also enforces this server-side via its jog/trajectory interlock).
function setControlMode(mode) {
  state.controlMode = mode;
  $("#mode-manual").classList.toggle("active", mode === "manual");
  $("#mode-programs").classList.toggle("active", mode === "programs");
  // Dim + block the jog pad outside manual mode; stop any motion when leaving it.
  $("#jog-axes").classList.toggle("jog-disabled", mode !== "manual");
  if (mode !== "manual") api("/api/jog/stop_all", { method: "POST" }).catch(() => {});
  updateProgramButtonsEnabled();
}

function updateProgramButtonsEnabled() {
  // Program actions need ROS ready, Programs mode, AND every axis homed
  // (absolute motion is blocked server-side until then).
  const ok = state.rosReady && state.controlMode === "programs" && state.allHomed;
  const reason = !state.allHomed ? "Home all axes first"
               : state.controlMode !== "programs" ? "Switch to Programs mode"
               : !state.rosReady ? "ROS bridge not ready" : "";
  for (const sel of ["#moveit-move", "#moveit-run"]) {
    const el = $(sel);
    if (!el) continue;
    el.disabled = !ok;
    el.title = ok ? "" : reason;
  }
}

async function refreshRosStatus() {
  const el = $("#ros-status");
  try {
    const s = await api("/api/ros/status");
    if (s.enabled && s.available) {
      const ready = s.action_server_ready ? "action server ready" : "waiting for action server";
      el.className = "text-sm text-emerald-400";
      el.textContent = `enabled — ${ready}`;
      state.rosReady = !!s.action_server_ready;
    } else {
      el.className = "text-sm text-yellow-400";
      el.textContent = s.rclpy ? "not enabled (start server with ARCTOS_ROS=1)"
                               : `unavailable: ${s.detail || "rclpy not installed"}`;
      state.rosReady = false;
    }
  } catch (e) {
    el.className = "text-sm text-red-400";
    el.textContent = "status error: " + e.message;
    state.rosReady = false;
  }
  updateProgramButtonsEnabled();
}

function initMoveit() {
  // Per-axis degree inputs.
  const host = $("#moveit-joints");
  host.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const wrap = document.createElement("label");
    wrap.className = "text-xs text-gray-400 flex items-center gap-1";
    wrap.innerHTML = `${ax.name}
      <input data-joint="${ax.name}" type="number" step="1"
             min="${ax.soft_limit_min}" max="${ax.soft_limit_max}" value="0"
             class="bg-gray-800 rounded px-2 py-1 w-20 text-sm" />`;
    host.appendChild(wrap);
  }

  $("#moveit-fill").addEventListener("click", () => {
    const live = state.axesLive || {};
    for (const inp of $$("#moveit-joints input")) {
      const s = live[inp.dataset.joint];
      if (s) inp.value = (s.degrees ?? 0).toFixed(1);
    }
  });

  $("#moveit-move").addEventListener("click", async () => {
    const joints_deg = {};
    for (const inp of $$("#moveit-joints input")) joints_deg[inp.dataset.joint] = parseFloat(inp.value) || 0;
    const duration_s = parseFloat($("#moveit-duration").value) || 3;
    moveitLog("sending joint goal…");
    try {
      const r = await api("/api/ros/move", { method: "POST", body: { joints_deg, duration_s } });
      moveitLog(r.accepted ? `done (error_code=${r.error_code})`
                           : `rejected: ${r.error_string}`,
                r.accepted && r.error_code === 0 ? "text-emerald-400" : "text-yellow-400");
    } catch (e) {
      moveitLog("move failed: " + e.message, "text-red-400");
    }
  });

  $("#moveit-run").addEventListener("click", async () => {
    const seg_time_s = parseFloat($("#moveit-segtime").value) || 2;
    moveitLog("running loaded program…");
    try {
      const r = await api("/api/ros/run_program", { method: "POST", body: { seg_time_s } });
      moveitLog(r.accepted ? `program done (error_code=${r.error_code})`
                           : `rejected: ${r.error_string}`,
                r.accepted && r.error_code === 0 ? "text-emerald-400" : "text-yellow-400");
    } catch (e) {
      moveitLog("run failed: " + e.message, "text-red-400");
    }
  });

  $("#ros-refresh").addEventListener("click", refreshRosStatus);
  refreshRosStatus();
}

function initTabs() {
  for (const tab of $$(".tabbtn")) {
    tab.addEventListener("click", () => {
      $$(".tabbtn").forEach(t => t.classList.remove("active"));
      tab.classList.add("active");
      $$(".tabpanel").forEach(p => p.classList.add("hidden"));
      document.getElementById("tab-" + tab.dataset.tab).classList.remove("hidden");
    });
  }
}

async function init() {
  try {
    state.cfg = await api("/api/config");
    renderJogAxes();
    renderConfig();
    initGripper();
    initHoming();
    initTeach();
    initMoveit();
    initTabs();
    connectWs();

    $("#speed").addEventListener("input", (e) => {
      state.speed = e.target.value / 100;
      $("#speed-val").textContent = e.target.value + "%";
    });
    $("#estop").addEventListener("click", () => api("/api/estop", { method: "POST" }));
    $("#home-all").addEventListener("click", async () => {
      if (!confirm("Home all axes in sequence? Each joint will seek its home switch.")) return;
      try {
        await api("/api/home/all", { method: "POST" });
        setStatus("homing all axes…", "text-amber-400");
      } catch (e) { setStatus("home all: " + e.message, "text-red-400"); }
    });
    $("#enable-btn").addEventListener("click", () => api("/api/enable", { method: "POST", body: { on: true } }));
    $("#disable-btn").addEventListener("click", () => api("/api/enable", { method: "POST", body: { on: false } }));
    $("#refresh-btn").addEventListener("click", () => api("/api/refresh", { method: "POST" }));
    $("#mode-manual").addEventListener("click", () => setControlMode("manual"));
    $("#mode-programs").addEventListener("click", () => setControlMode("programs"));
    setControlMode("manual");
  } catch (e) {
    setStatus("init failed: " + e.message, "text-red-400");
  }
}
init();
