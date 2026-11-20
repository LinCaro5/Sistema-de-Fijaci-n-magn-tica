const API = {
  telemetry: "api/get_data.php",
  command: "api/send_command.php",
  refreshMs: 15000,
  onlineThresholdMs: 45000
};

let forceChart = null;
let latestFeedTimestamp = null;
let isSendingCommand = false;

const $ = (id) => document.getElementById(id);
const number = (value, digits = 2) => {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "--";
};

function setStatus(id, text, type = "neutral") {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  el.className = `status-value ${type}`;
}

function initChart() {
  const canvas = $("force-chart");
  if (!canvas || !window.Chart) return;
  forceChart = new Chart(canvas, {
    type: "line",
    data: {
      labels: [],
      datasets: [
        { label: "Fuerza X (N)", data: [], borderColor: "#3867d6", backgroundColor: "rgba(56,103,214,.08)", borderWidth: 2, pointRadius: 2, tension: .28 },
        { label: "Fuerza Y (N)", data: [], borderColor: "#8066d8", backgroundColor: "rgba(128,102,216,.06)", borderWidth: 2, pointRadius: 2, tension: .28 }
      ]
    },
    options: {
      responsive: true, maintainAspectRatio: false, interaction: { mode: "index", intersect: false },
      plugins: { legend: { position: "bottom", labels: { usePointStyle: true, boxWidth: 7, color: "#7b8497", font: { family: "Inter", size: 10 } } } },
      scales: {
        x: { title: { display: true, text: "Hora de registro", color: "#8993a4", font: { size: 10 } }, grid: { display: false }, ticks: { color: "#9aa3b2", maxTicksLimit: 7, font: { size: 9 } } },
        y: { title: { display: true, text: "Fuerza (N)", color: "#8993a4", font: { size: 10 } }, grid: { color: "#eef1f6" }, ticks: { color: "#9aa3b2", font: { size: 9 } } }
      }
    }
  });
}

function updateChart(feeds) {
  if (!forceChart) return;
  const valid = (feeds || []).filter(f => f.field1 !== null && f.field1 !== "" && f.field2 !== null && f.field2 !== "");
  forceChart.data.labels = valid.map(f => {
    const d = new Date((f.created_at || "").replace(" ", "T") + (String(f.created_at || "").endsWith("Z") ? "" : "Z"));
    return Number.isNaN(d.getTime()) ? "--:--:--" : d.toLocaleTimeString("es-CO", { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  });
  forceChart.data.datasets[0].data = valid.map(f => Number(f.field2)); // Field 2 = Fx
  forceChart.data.datasets[1].data = valid.map(f => Number(f.field1)); // Field 1 = Fy
  forceChart.update("none");
}

function updateConnection(timestamp) {
  const pill = $("connection-pill");
  const text = $("connection-text");
  const device = $("device-status");
  let age = timestamp ? Date.now() - new Date(timestamp.replace(" ", "T") + (timestamp.endsWith("Z") ? "" : "Z")).getTime() : Infinity;
  const online = Number.isFinite(age) && age >= -60000 && age <= API.onlineThresholdMs;
  pill.classList.toggle("online", online);
  pill.classList.toggle("offline", !online);
  text.textContent = online ? "Dispositivo en línea" : "Sin telemetría reciente";
  setStatus("device-status", online ? "En línea" : "Sin conexión reciente", online ? "success" : "danger");
}

function renderTelemetry(data) {
  const latest = data.latest || {};
  const feeds = data.feeds || [];
  const fy = latest.field1;
  const fx = latest.field2;
  const magnitude = latest.field3;
  const angle = latest.field4;
  const position = latest.field5;
  const moving = Number(latest.field6) === 1;
  const zDetected = Number(latest.field7) === 1;
  const lost = latest.field8;

  $("overview-fx").textContent = number(fx);
  $("overview-fy").textContent = number(fy);
  $("overview-magnitude").textContent = number(magnitude);
  $("overview-position").textContent = number(position, 1);
  $("force-fx").textContent = number(fx);
  $("force-fy").textContent = number(fy);
  $("force-magnitude").textContent = number(magnitude);
  $("force-angle").textContent = number(angle, 1);
  $("system-position").textContent = number(position, 1);
  $("lost-samples").textContent = lost === null || lost === undefined || lost === "" ? "--" : String(lost);

  const pos = Number(position);
  const clamped = Number.isFinite(pos) ? Math.max(0, Math.min(30, pos)) : 0;
  $("position-progress").style.width = `${(clamped / 30) * 100}%`;
  $("overview-position-label").textContent = Number.isFinite(pos) ? (Math.abs(pos - 0) < 1.5 ? "Cercana a posición A" : Math.abs(pos - 30) < 1.5 ? "Cercana a posición B" : "Entre posiciones A y B") : "Posición mecánica";

  $("overview-motion").textContent = moving ? "En movimiento" : "Detenido";
  $("system-motion").textContent = moving ? "En movimiento" : "Detenido";
  $("motion-dot").classList.toggle("moving", moving);
  setStatus("overview-motion", moving ? "En movimiento" : "Detenido", moving ? "warning" : "success");

  $("overview-z").textContent = zDetected ? "Detectado" : "No detectado";
  const zBadge = $("z-badge");
  zBadge.textContent = zDetected ? "Patrón detectado" : "Patrón no detectado";
  zBadge.className = `state-badge ${zDetected ? "detected" : "clear"}`;

  // Fields 9 and 10 are optional extensions for Z signal and response ratio.
  $("z-signal").textContent = latest.field9 ? number(latest.field9, 1) : "--";
  $("z-ratio").textContent = latest.field10 ? number(latest.field10, 3) : "--";

  const timestamp = latest.created_at || null;
  latestFeedTimestamp = timestamp;
  $("last-update").textContent = timestamp ? new Date(timestamp.replace(" ", "T") + (timestamp.endsWith("Z") ? "" : "Z")).toLocaleString("es-CO") : "Sin datos";
  updateConnection(timestamp);
  updateChart(feeds);
}

async function loadTelemetry() {
  try {
    const response = await fetch(`${API.telemetry}?_=${Date.now()}`, { cache: "no-store" });
    const data = await response.json();
    if (!response.ok || !data.success) throw new Error(data.error || "No fue posible consultar la telemetría.");
    renderTelemetry(data);
  } catch (error) {
    $("connection-text").textContent = "Error de comunicación";
    $("connection-pill").classList.remove("online");
    $("connection-pill").classList.add("offline");
    setStatus("device-status", "Error de consulta", "danger");
    console.error("Error consultando telemetría:", error);
  }
}

async function sendCommand(command) {
  if (isSendingCommand) return;
  const labels = { 0: "STOP", 1: "posición A", 2: "posición B" };
  const confirmed = command === 0
    ? window.confirm("¿Enviar el comando STOP al sistema?")
    : window.confirm(`¿Enviar el comando para mover el sistema a ${labels[command]}?`);
  if (!confirmed) return;

  isSendingCommand = true;
  const buttons = document.querySelectorAll("[data-command]");
  buttons.forEach(button => button.disabled = true);
  $("command-feedback").textContent = `Enviando comando ${labels[command]}...`;
  try {
    const response = await fetch(API.command, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command })
    });
    const result = await response.json();
    if (!response.ok || !result.success) throw new Error(result.error || "No fue posible enviar el comando.");
    $("command-feedback").textContent = `Comando ${labels[command]} enviado a ThingSpeak. Entrada registrada: ${result.entry_id}. La ejecución depende de que el dispositivo reciba y acepte la orden.`;
  } catch (error) {
    $("command-feedback").textContent = `No se pudo enviar el comando: ${error.message}`;
  } finally {
    isSendingCommand = false;
    buttons.forEach(button => button.disabled = false);
  }
}

document.addEventListener("DOMContentLoaded", () => {
  initChart();
  loadTelemetry();
  window.setInterval(loadTelemetry, API.refreshMs);
  document.querySelectorAll("[data-command]").forEach(button => {
    button.addEventListener("click", () => sendCommand(Number(button.dataset.command)));
  });

  const links = document.querySelectorAll(".nav-link");
  const observer = new IntersectionObserver(entries => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        links.forEach(link => link.classList.toggle("active", link.getAttribute("href") === `#${entry.target.id}`));
      }
    });
  }, { rootMargin: "-25% 0px -60% 0px" });
  document.querySelectorAll(".dashboard-section").forEach(section => observer.observe(section));
});
