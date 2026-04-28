// arctos-pi minimal jog UI. Vanilla JS, no build step.

const state = {
  axes: [], cfg: null, speed: 0.25, ws: null, lastPong: 0,
  gripperDragging: false,
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
      <div class="pos w-28 text-right text-lg font-mono" id="pos-${ax.can_id}">0.00°</div>
      <button class="btn jogbtn bg-gray-700 rounded-lg flex-1" data-can="${ax.can_id}" data-dir="1">+</button>
    `;
    host.appendChild(row);
  }
  // Bind hold-to-jog
  for (const b of $$(".jogbtn")) {
    const start = (e) => {
      e.preventDefault();
      b.classList.add("active");
      const can = parseInt(b.dataset.can, 10);
      const dir = parseInt(b.dataset.dir, 10);
      api("/api/jog/start", { method: "POST", body: { can_id: can, direction: dir, speed_pct: state.speed } })
        .catch((err) => setStatus("jog err: " + err.message, "text-red-400"));
    };
    const stop = (e) => {
      if (!b.classList.contains("active")) return;
      e.preventDefault();
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

function renderStatus(axes) {
  const body = $("#status-body");
  body.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const s = axes[ax.name] || {};
    const tr = document.createElement("tr");
    tr.className = "border-t border-gray-800";
    tr.innerHTML = `
      <td class="py-1">${ax.name} <span class="text-gray-500 text-xs">id ${ax.can_id}</span></td>
      <td class="text-right pos">${(s.degrees ?? 0).toFixed(2)}°</td>
      <td class="text-right pos text-gray-400">${s.pulses ?? 0}</td>
      <td class="text-right">${s.enabled ? "✓" : "—"}</td>
    `;
    body.appendChild(tr);
    const posEl = document.getElementById(`pos-${ax.can_id}`);
    if (posEl) posEl.textContent = (s.degrees ?? 0).toFixed(2) + "°";
  }
}

function renderGripper(g) {
  if (!g || !g.enabled) return;
  $("#gripper-pos").textContent = String(g.position ?? 0);
  if (!state.gripperDragging) {
    $("#gripper-slider").value = String(g.position ?? 0);
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

  slider.addEventListener("pointerdown", () => { state.gripperDragging = true; });
  slider.addEventListener("pointerup", () => { state.gripperDragging = false; });
  slider.addEventListener("pointercancel", () => { state.gripperDragging = false; });
  slider.addEventListener("input", () => {
    $("#gripper-pos").textContent = slider.value;
  });
  slider.addEventListener("change", async () => {
    try {
      await api("/api/gripper", { method: "POST", body: { position: parseInt(slider.value, 10) } });
    } catch (e) {
      setStatus("gripper err: " + e.message, "text-red-400");
    }
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

function renderConfig() {
  const host = $("#cfg-body");
  host.innerHTML = "";
  for (const ax of state.cfg.axes) {
    const card = document.createElement("div");
    card.className = "bg-gray-900 rounded-lg p-3 flex items-center gap-3";
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
      renderStatus(msg.axes);
      renderGripper(msg.gripper);
      renderTeachSummary(msg.teach);
    }
    if (msg.type === "pong") state.lastPong = msg.t;
  };
  ws.onclose = () => {
    setStatus("disconnected, retrying…", "text-yellow-400");
    setTimeout(connectWs, 1000);
  };
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
    initTeach();
    initTabs();
    connectWs();

    $("#speed").addEventListener("input", (e) => {
      state.speed = e.target.value / 100;
      $("#speed-val").textContent = e.target.value + "%";
    });
    $("#estop").addEventListener("click", () => api("/api/estop", { method: "POST" }));
    $("#enable-btn").addEventListener("click", () => api("/api/enable", { method: "POST", body: { on: true } }));
    $("#disable-btn").addEventListener("click", () => api("/api/enable", { method: "POST", body: { on: false } }));
    $("#refresh-btn").addEventListener("click", () => api("/api/refresh", { method: "POST" }));
  } catch (e) {
    setStatus("init failed: " + e.message, "text-red-400");
  }
}
init();
