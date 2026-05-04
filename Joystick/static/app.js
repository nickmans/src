const wsStatusEl = document.getElementById("wsStatus");
const btStatusEl = document.getElementById("btStatus");
const angleValueEl = document.getElementById("angleValue");
const speedValueEl = document.getElementById("speedValue");
const joystickEl = document.getElementById("joystick");
const stickEl = document.getElementById("stick");
const joyModeBtn = document.getElementById("joyModeBtn");
const estopBtn = document.getElementById("estopBtn");

let ws = null;
let reconnectTimer = null;
let activePointerId = null;

const joystickState = {
  x: 0,
  y: 0,
};

function updatePill(el, isConnected, textConnected, textDisconnected) {
  el.classList.toggle("connected", isConnected);
  el.classList.toggle("disconnected", !isConnected);
  el.textContent = isConnected ? textConnected : textDisconnected;
}

function sendWsMessage(payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return;
  }
  ws.send(JSON.stringify(payload));
}

function connectWebSocket() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const url = `${proto}://${window.location.host}/ws`;

  ws = new WebSocket(url);

  ws.addEventListener("open", () => {
    updatePill(wsStatusEl, true, "WS Connected", "WS Disconnected");
  });

  ws.addEventListener("close", () => {
    updatePill(wsStatusEl, false, "WS Connected", "WS Disconnected");
    updatePill(btStatusEl, false, "LINK Connected", "LINK Disconnected");

    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
    }
    reconnectTimer = setTimeout(connectWebSocket, 1000);
  });

  ws.addEventListener("message", (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "telemetry") {
        angleValueEl.textContent = `${msg.angle}\u00b0`;
        speedValueEl.textContent = String(msg.speed);
        const transport = (msg.transport || "link").toUpperCase();
        updatePill(
          btStatusEl,
          !!msg.link_connected,
          `${transport} Connected`,
          `${transport} Disconnected`
        );
      }
    } catch (_err) {
      // Ignore malformed status payloads from server.
    }
  });
}

function clampToCircle(nx, ny) {
  const mag = Math.hypot(nx, ny);
  if (mag <= 1) {
    return { x: nx, y: ny };
  }
  return { x: nx / mag, y: ny / mag };
}

function moveStickVisual(x, y) {
  const radiusPx = joystickEl.clientWidth / 2;
  const stickTravel = radiusPx * 0.72;
  const px = x * stickTravel;
  const py = -y * stickTravel;
  stickEl.style.transform = `translate(calc(-50% + ${px}px), calc(-50% + ${py}px))`;
}

function updateJoystickFromEvent(clientX, clientY) {
  const rect = joystickEl.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const radius = rect.width / 2;

  const dx = clientX - cx;
  const dy = cy - clientY;

  const nx = dx / radius;
  const ny = dy / radius;
  const clamped = clampToCircle(nx, ny);

  joystickState.x = clamped.x;
  joystickState.y = clamped.y;

  moveStickVisual(joystickState.x, joystickState.y);
  sendWsMessage({ type: "joystick", x: joystickState.x, y: joystickState.y });
}

function releaseJoystick() {
  joystickState.x = 0;
  joystickState.y = 0;
  moveStickVisual(0, 0);
  sendWsMessage({ type: "joystick", x: 0, y: 0 });
}

joystickEl.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  activePointerId = event.pointerId;
  joystickEl.setPointerCapture(activePointerId);
  updateJoystickFromEvent(event.clientX, event.clientY);
});

joystickEl.addEventListener("pointermove", (event) => {
  if (activePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  updateJoystickFromEvent(event.clientX, event.clientY);
});

function onPointerEnd(event) {
  if (activePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  activePointerId = null;
  releaseJoystick();
}

joystickEl.addEventListener("pointerup", onPointerEnd);
joystickEl.addEventListener("pointercancel", onPointerEnd);
joystickEl.addEventListener("lostpointercapture", () => {
  if (activePointerId !== null) {
    activePointerId = null;
    releaseJoystick();
  }
});

joyModeBtn.addEventListener("click", () => {
  sendWsMessage({ type: "enable_joy" });
  joyModeBtn.textContent = "Joystick Mode Requested";
  window.setTimeout(() => {
    joyModeBtn.textContent = "Enable Joystick Mode";
  }, 1200);
});

estopBtn.addEventListener("click", () => {
  releaseJoystick();
  sendWsMessage({ type: "estop" });
});

window.addEventListener("beforeunload", () => {
  releaseJoystick();
});

connectWebSocket();