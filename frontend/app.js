// arctos-pi minimal jog UI. Vanilla JS, no build step.

const state = { axes: [], cfg: null, speed: 0.25, ws: null, lastPong: 0 };
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
    if (msg.type === "state") renderStatus(msg.axes);
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
