"use strict";
const el = (id) => document.getElementById(id);
let state = null;
let commandBusy = false;
let resetBusy = false;
let commandQueue = Promise.resolve();
let polling = false;
let lastAckTime = "";
let commandPosting = false;
const barcodeActions = ["left", "right", "straight", "uturn"];
let barcodeEdited = false;
let barcodePosting = false;
let barcodeError = "";
function setFlag(id, value, fresh) {
  const node = el(id);
  node.className = "flag" + (fresh ? value ? " good" : " bad" : "");
  node.textContent = fresh ? value ? "Connected" : "Offline" : "Unconfirmed";
  if (id === "subscribed" && fresh && value) node.textContent = "Subscribed";
}
function draw(data) {
  state = data;
  el("broker-address").textContent = `${data.broker}:${data.broker_port}`;
  el("topic-root").textContent = data.root;
  el("live-monitor").classList.toggle("online", data.broker_connected);
  el("start").disabled = !data.start_allowed || commandBusy || barcodePosting;
  el("stop").disabled = !data.stop_allowed;
  el("reset").disabled = !data.reset_allowed || resetBusy;
  el("ready-dot").classList.toggle("online", data.start_allowed);
  let title = "Waiting for the Pico";
  let description = "Waiting for a live heartbeat. Stored broker messages do not confirm the Pico is online.";
  if (!data.broker_connected) {
    title = "Broker unavailable";
    description = "Check the broker address, listener and firewall. The dashboard retries automatically.";
  } else if (!data.monitor_ready) {
    description = "Waiting for the dashboard’s MQTT subscriptions.";
  } else if (data.telemetry?.controller?.fresh && data.telemetry.controller.data.start_block) {
    title = "START blocked by controller";
    description = data.telemetry.controller.data.start_block_label;
  } else if (data.start_allowed) {
    title = "Pico ready for commands";
    description = "The live heartbeat confirms Wi-Fi, MQTT and the command subscription.";
  } else if (data.pico.age !== null && !data.pico.fresh) {
    title = "Heartbeat is stale";
    description = "No live signal for over 15 seconds. START is paused; STOP remains available.";
  } else if (data.pico.fresh) {
    title = "Pico connection not ready";
    description = "Wait for all three connection flags before sending START.";
  }
  el("pico-state").textContent = title;
  el("pico-description").textContent = description;
  el("notice").hidden = !data.error;
  el("notice").textContent = data.error;
  el("heartbeat-age").textContent = data.pico.age === null ? "—" : `${Math.floor(data.pico.age)}s ago`;
  el("heartbeat-age").closest(".heartbeat-card").classList.toggle("live", data.pico.fresh);
  for (const id of ["wifi", "mqtt", "subscribed"]) setFlag(id, data.pico[id], data.pico.fresh && data.broker_connected);
  el("status-hex").textContent = data.pico.hex ?? "—";
  drawTelemetry(data);
  drawBarcodeMap(data);
  el("drop-status").textContent = data.pico.hex === null ? "Waiting for data" : data.pico.tx_drop || data.pico.rx_drop ? "DROP FLAG SET" : "NO DROP FLAGS";
  if (data.last_ack && data.last_ack.time !== lastAckTime && !commandPosting) {
    const ack = data.last_ack;
    lastAckTime = ack.time;
    el("command-feedback").className = "command-feedback" + (ack.rejected ? " error" : "");
    el("command-feedback").textContent = `Pico ACK: ${ack.command.toUpperCase()} ${ack.rejected ? "rejected" : "accepted for processing"}.`;
  }
  if (data.last_ack?.rejected && !commandPosting && el("command-feedback").textContent.startsWith("Pico ACK:")) {
    const diagnostics = data.telemetry?.controller;
    const reason = data.last_ack.command === "start" && diagnostics?.fresh && diagnostics.data.last_start_reject ? `${diagnostics.data.last_reject_label}. ${diagnostics.data.last_reject_hint}` : "Reason unavailable: update the Pico firmware and inspect controller diagnostics.";
    el("command-feedback").textContent = `Pico ACK: ${data.last_ack.command.toUpperCase()} rejected — ${reason}`;
  }
}
async function refresh() {
  if (polling) return;
  polling = true;
  try {
    const result = await fetch("/api/state", { cache: "no-store", signal: AbortSignal.timeout(4000) });
    if (!result.ok) throw new Error("Dashboard connection failed");
    draw(await result.json());
  } catch (error) {
    state = null;
    drawTelemetry(null);
    drawBarcodeMap(null);
    el("start").disabled = el("stop").disabled = el("reset").disabled = true;
    el("live-monitor").classList.remove("online");
    el("ready-dot").classList.remove("online");
    for (const id of ["wifi", "mqtt", "subscribed"]) setFlag(id, false, false);
    el("heartbeat-age").textContent = "—";
    el("heartbeat-age").closest(".heartbeat-card").classList.remove("live");
    el("pico-state").textContent = "Dashboard server unavailable";
    el("pico-description").textContent = "Keep app.py running, then refresh this page.";
    el("notice").hidden = false;
    el("notice").textContent = "Could not reach the local dashboard server.";
  } finally {
    polling = false;
  }
}
function send(command) {
  if (!state || (command === "start" && (commandBusy || barcodePosting)) || (command === "reset" && resetBusy)) return;
  if (command === "reset") { resetBusy = true; el("reset").disabled = true; }
  if (command === "start") { commandBusy = true; el("start").disabled = true; }
  // Preserve operator click order, including STOP clicked while START is posting.
  commandQueue = commandQueue.then(() => performSend(command));
}
async function performSend(command) {
  const token = state?.csrf ?? "";
  commandPosting = true;
  el("command-feedback").className = "command-feedback";
  el("command-feedback").textContent = `Sending ${command.toUpperCase()}…`;
  try {
    const response = await fetch("/api/command", {
      method: "POST", headers: { "Content-Type": "application/json", "X-Dashboard-Token": token },
      body: JSON.stringify({ command }), signal: AbortSignal.timeout(5000)
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "Command was not accepted");
    el("command-feedback").textContent = `${command.toUpperCase()} queued for the broker. Waiting for a Pico acknowledgment.`;
  } catch (error) {
    el("command-feedback").className = "command-feedback error";
    el("command-feedback").textContent = error.name === "TimeoutError" ? "Delivery is uncertain. Check the ACK and USB serial before retrying." : error.message;
  } finally {
    commandPosting = false;
    if (command === "start") commandBusy = false;
    if (command === "reset") resetBusy = false;
    await refresh();
  }
}
el("start").addEventListener("click", () => send("start"));
el("stop").addEventListener("click", () => send("stop"));
el("reset").addEventListener("click", () => send("reset"));

function barcodeDraft() {
  return Object.fromEntries(barcodeActions.map(action => [action, el("barcode-" + action).value]));
}

function sameMapping(first, second) {
  return Boolean(first && second && barcodeActions.every(action => first[action] === second[action]));
}

function drawBarcodeMap(data) {
  const report = data?.telemetry?.barcode_map;
  const active = report?.data;
  const request = data?.barcode_request;
  const pending = barcodePosting || request?.status === "pending";
  if (active && (!barcodeEdited || (!pending && sameMapping(barcodeDraft(), active)))) {
    barcodeEdited = false;
    for (const action of barcodeActions) el("barcode-" + action).value = active[action];
  }
  for (const action of barcodeActions) el("barcode-" + action).disabled = !report?.fresh || pending;
  const draft = barcodeDraft();
  const duplicate = new Set(Object.values(draft)).size !== barcodeActions.length;
  const changed = Boolean(active && !sameMapping(draft, active));
  el("barcode-apply").disabled = pending || !data?.barcode_map_allowed || duplicate || !changed;

  const activeRow = el("barcode-active");
  activeRow.hidden = !active || (!changed && report.fresh);
  if (active) activeRow.textContent = `${report.fresh ? "Active" : "Last reported"} mapping · Left ${active.left} · Right ${active.right} · Straight ${active.straight} · U-turn ${active.uturn}`;
  const status = el("barcode-map-status");
  let message = "";
  if (pending) message = "Waiting for the Pico to confirm the mapping…";
  else if (barcodeError) message = barcodeError;
  else if (!report?.fresh) message = "Waiting for a live mapping from the Pico.";
  else if (duplicate) message = "Use a different letter for each action.";
  else if (request?.status === "unconfirmed" && changed && sameMapping(request.mapping, draft)) {
    message = "The Pico did not confirm this mapping. Check the active mapping and try again while stopped.";
  } else if (!data.barcode_map_allowed) message = "Stop the car to apply a mapping.";
  else if (changed) message = "Mapping changes are ready to apply.";
  else if (request?.status === "applied" && sameMapping(request.mapping, active)) message = "Mapping confirmed by the Pico.";
  else message = "The mapping stays active until the Pico restarts.";
  if (status.textContent !== message) status.textContent = message;
  status.classList.toggle("error", Boolean(barcodeError || (report?.fresh && duplicate)));
}

for (const action of barcodeActions) {
  const select = el("barcode-" + action);
  for (const letter of "ABCDEFGHIJKLMNOPQRSTUVWXYZ") select.add(new Option(letter, letter));
  select.value = "";
  select.addEventListener("change", () => {
    barcodeEdited = true;
    barcodeError = "";
    drawBarcodeMap(state);
  });
}
el("barcode-map-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!state || el("barcode-apply").disabled) return;
  const mapping = barcodeDraft();
  barcodePosting = true;
  el("start").disabled = true;
  barcodeError = "";
  drawBarcodeMap(state);
  try {
    const response = await fetch("/api/barcode-map", {
      method: "POST", headers: { "Content-Type": "application/json", "X-Dashboard-Token": state.csrf },
      body: JSON.stringify(mapping), signal: AbortSignal.timeout(5000)
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "The mapping could not be sent.");
    if (state) state.barcode_request = result.barcode_request;
  } catch (error) {
    barcodeError = error.name === "TimeoutError" ? "Delivery is uncertain. Check the active mapping before retrying." : error.message;
  } finally {
    barcodePosting = false;
    drawBarcodeMap(state);
    await refresh();
  }
});

refresh();
setInterval(refresh, 1000);

function text(id, value) { el(id).textContent = value ?? "—"; }
function faultList(id, labels, available) {
  const node = el(id);
  node.textContent = !available ? "Fault status unavailable" : labels.length ? labels.join(" · ") : "No reported faults";
  node.classList.toggle("has-fault", available && labels.length > 0);
}
function drawTelemetry(data) {
  const topics = data?.telemetry ?? {};
  for (const name of ["vehicle", "motion", "line", "terrain", "obstacle"]) {
    const sample = topics[name];
    const card = el(name + "-card");
    const fresh = Boolean(sample?.fresh);
    card.classList.toggle("stale", Boolean(sample?.data) && !fresh);
    card.classList.toggle("empty", !sample?.data);
    const badge = el(name + "-freshness");
    badge.className = "sample-status" + (fresh ? " fresh" : sample?.data ? " stale" : "");
    badge.textContent = !sample?.data ? "No data" : fresh ? "Live" : `Stale · ${Math.floor(sample.age)}s`;
  }
  const diagnostic = topics.controller;
  const controller = diagnostic?.data;
  text("start-blocker", controller ? diagnostic.fresh ? controller.start_block_label : "Controller diagnostics stale" : "Controller diagnostics unavailable");
  text("start-hint", controller && diagnostic.fresh ? controller.start_hint : "Use the matching Pico diagnostics firmware and dashboard to report the START blocker.");
  text("ir-age", controller ? controller.line_age_ms === null ? "No valid sample" : controller.line_age_ms + " ms" : null);
  text("line-stop", controller ? controller.line_stop ? "Latched" : "No" : null);
  text("controller-detail", controller ? `IR darkness L/C/R: ${controller.left_dark} / ${controller.centre_dark} / ${controller.right_dark}. Target RPM L/R: ${controller.left_target_rpm} / ${controller.right_target_rpm}. Telemetry drops: ${controller.telemetry_drops}.` : "Waiting for live controller diagnostics.");
  const vehicle = topics.vehicle?.data;
  const motion = topics.motion?.data;
  const line = topics.line?.fresh ? topics.line.data : null;
  const terrain = topics.terrain?.data;
  const obstacle = topics.obstacle?.data;
  text("mission-state", vehicle?.state.replaceAll("_", " "));
  const descriptions = {
    IDLE: "Controller is stopped or waiting for START.",
    FOLLOW_LINE: "Controller is applying line-following recommendations.",
    FAULT: "Controller has stopped on a fault. Inspect the readings before restarting.",
    TURNING: "Navigating a barcode junction and checking the branch tape.", AVOIDING: "An avoidance mission state was reported.",
    RECOVER_LINE: "A line-recovery mission state was reported.",
    RETRY_BARCODE: "Backing up in a straight line for one more barcode read."
  };
  const stopped = vehicle?.state === "IDLE" || vehicle?.state === "FAULT";
  const stopReason = stopped && diagnostic?.fresh && controller.state === vehicle.state
    ? controller.stop_reason_label : null;
  text("mission-description", vehicle ? topics.vehicle.fresh ? stopReason ?? descriptions[vehicle.state] : "Last report only. Waiting for live mission telemetry." : "Waiting for a mission-state report.");
  faultList("vehicle-faults", vehicle?.fault_labels ?? [], Boolean(vehicle));
  const sonar = topics.sonar?.fresh ? topics.sonar.data : null;
  const scanning = sonar?.phase === "checking side clearance" || sonar?.phase === "scanning ahead";
  const bypassing = ["turning off the tape", "leaving the tape", "aligning for the bypass",
    "passing the obstacle", "turning toward the tape", "searching for the tape", "centring on the tape",
    "realigning with the tape", "confirming the tape"].includes(sonar?.phase);
  const forward = typeof sonar?.forward_mm === "number" ? sonar.forward_mm + " mm" : sonar?.forward_mm ?? "Unavailable";
  const detour = sonar?.failure ? " · " + sonar.failure + " while " + sonar.failed_phase : "";
  let sonarActivity = "Latest forward sonar: " + forward;
  if (scanning) sonarActivity = "Sonar scan in progress";
  else if (bypassing) sonarActivity = "Bypass in progress · " + sonar.phase;
  el("sonar-detail").hidden = sonar === null;
  text("sonar-detail", sonar ? sonarActivity + detour : null);
  faultList("motor-faults", motion?.fault_labels ?? [], Boolean(motion));
  if (motion?.faults & 2 && controller?.timing_gap_us != null) {
    el("motor-faults").textContent += ` · Gap ${(controller.timing_gap_us/1000).toFixed(2)} ms / limit ${(controller.timing_limit_us/1000).toFixed(2)} ms`;
  }
  for (const side of ["left", "right"]) {
    text(side + "-rpm", motion?.[side + "_rpm"]);
    text(side + "-pwm", motion ? motion[side + "_pwm"] + "%" : null);
    el(side + "-pwm-bar").value = motion?.[side + "_pwm"] ?? 0;
    text(side + "-progress", motion?.[side + "_progress"]);
    const black = line?.[side + "_black"];
    text(side + "-ir", line ? black ? "Black detected" : "No black" : null);
    el(side + "-ir-light").classList.toggle("black", Boolean(black));
  }
  text("motion-mode", motion?.mode_name);
  text("speed-level", motion?.speed_level);
  text("barcode", line?.barcode);
  text("centre-ir", line ? line.centre_black ? "Black" : "Not set" : null);
  text("terrain-height", terrain?.height_mm);
  text("terrain-max-height", typeof terrain?.max_height_mm === "number" ? terrain.max_height_mm + " mm" : null);
  text("terrain-phase", terrain?.phase_label);
  for (const [id, key] of [["nearest", "nearest_mm"], ["width", "width_mm"], ["left", "left_mm"], ["right", "right_mm"]]) {
    const value = obstacle?.[key];
    text("obstacle-" + id, typeof value === "number" ? value + " mm" : value);
  }
  const history = data?.motion_history ?? [];
  const maximum = Math.max(30, ...history.flatMap(p => [p.left_rpm, p.right_rpm]));
  for (const side of ["left", "right"]) {
    const path = history.map((p, i) => `${i ? "L" : "M"}${(i * 400 / Math.max(history.length - 1, 1)).toFixed(1)},${(66 - p[side + "_rpm"] * 60 / maximum).toFixed(1)}`).join(" ");
    el(side + "-rpm-path").setAttribute("d", path);
  }
  const events = el("events");
  events.replaceChildren();
  for (const event of data?.events ?? []) {
    const row = document.createElement("li");
    const time = document.createElement("time");
    time.dateTime = event.time;
    time.textContent = new Date(event.time).toLocaleTimeString();
    const message = document.createElement("span");
    message.textContent = event.message;
    row.append(time, message); events.append(row);
  }
  if (!events.children.length) {
    const empty = document.createElement("li");empty.textContent = data ? "Waiting for events." : "Dashboard connection unavailable.";events.append(empty);
  }
}
