const wsStatusEl = document.getElementById("wsStatus");
const btStatusEl = document.getElementById("btStatus");
const angleValueEl = document.getElementById("angleValue");
const speedValueEl = document.getElementById("speedValue");
const joystickEl = document.getElementById("joystick");
const stickEl = document.getElementById("stick");
const joyModeBtn = document.getElementById("joyModeBtn");
const spin2Btn = document.getElementById("spin2Btn");
const spin1Btn = document.getElementById("spin1Btn");
const spin0Btn = document.getElementById("spin0Btn");
const focusBtn = document.getElementById("focusBtn");
const estopBtn = document.getElementById("estopBtn");
const operatorNoticeEl = document.getElementById("operatorNotice");

let ws = null;
let reconnectTimer = null;
let activePointerId = null;
let holdSendTimer = null;

const HOLD_RESEND_MS = 50;
let joyModeEnabled = false;
let focusEnabled = false;
let controllerBusy = false;
let lastFocusRequestMs = 0;

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
  if (controllerBusy) {
    return false;
  }
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return false;
  }
  ws.send(JSON.stringify(payload));
  return true;
}

function renderOperatorNotice() {
  if (!controllerBusy) {
    operatorNoticeEl.hidden = true;
    operatorNoticeEl.textContent = "";
    return;
  }

  operatorNoticeEl.hidden = false;
  operatorNoticeEl.textContent = "Controller busy: another user currently has robot control.";
}

function renderJoyModeButton() {
  joyModeBtn.textContent = joyModeEnabled
    ? "Disable Joystick Mode"
    : "Enable Joystick Mode";
  const disableControls = controllerBusy;
  joyModeBtn.disabled = disableControls;
  estopBtn.disabled = disableControls;
  spin2Btn.disabled = disableControls || !joyModeEnabled;
  spin1Btn.disabled = disableControls || !joyModeEnabled;
  spin0Btn.disabled = disableControls || !joyModeEnabled;
  focusBtn.disabled = disableControls;
  joystickEl.classList.toggle("disabled", disableControls);
}

function renderFocusButton() {
  focusBtn.textContent = focusEnabled ? "focus off" : "focus on";
}

function requestFocusToggle() {
  const nowMs = Date.now();
  if (nowMs - lastFocusRequestMs < 200) {
    return;
  }
  lastFocusRequestMs = nowMs;

  const nextFocusEnabled = !focusEnabled;
  if (sendWsMessage({ type: "focus", enabled: nextFocusEnabled })) {
    focusEnabled = nextFocusEnabled;
    renderFocusButton();
  }
}

function startHoldResendLoop() {
  if (holdSendTimer) {
    clearInterval(holdSendTimer);
  }

  holdSendTimer = setInterval(() => {
    if (activePointerId === null) {
      return;
    }
    sendWsMessage({ type: "joystick", x: joystickState.x, y: joystickState.y });
  }, HOLD_RESEND_MS);
}

function stopHoldResendLoop() {
  if (!holdSendTimer) {
    return;
  }
  clearInterval(holdSendTimer);
  holdSendTimer = null;
}

function connectWebSocket() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const url = `${proto}://${window.location.host}/ws`;

  ws = new WebSocket(url);

  ws.addEventListener("open", () => {
    controllerBusy = false;
    renderOperatorNotice();
    updatePill(wsStatusEl, true, "WS Connected", "WS Disconnected");
    renderJoyModeButton();
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
      if (msg.type === "busy") {
        controllerBusy = true;
        renderOperatorNotice();
        releaseJoystick();
        renderJoyModeButton();
      } else if (msg.type === "telemetry") {
        angleValueEl.textContent = `${msg.angle}\u00b0`;
        speedValueEl.textContent = String(msg.speed);
        joyModeEnabled = !!msg.joy_enabled;
        focusEnabled = !!msg.focus_enabled;
        renderJoyModeButton();
        renderFocusButton();
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
  if (controllerBusy) {
    return;
  }
  event.preventDefault();
  activePointerId = event.pointerId;
  joystickEl.setPointerCapture(activePointerId);
  startHoldResendLoop();
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
  stopHoldResendLoop();
  activePointerId = null;
  releaseJoystick();
}

joystickEl.addEventListener("pointerup", onPointerEnd);
joystickEl.addEventListener("pointercancel", onPointerEnd);
joystickEl.addEventListener("lostpointercapture", () => {
  if (activePointerId !== null) {
    stopHoldResendLoop();
    activePointerId = null;
    releaseJoystick();
  }
});

joyModeBtn.addEventListener("click", () => {
  if (controllerBusy) {
    return;
  }
  const nextJoyModeEnabled = !joyModeEnabled;
  sendWsMessage({ type: nextJoyModeEnabled ? "enable_joy" : "disable_joy" });
  joyModeEnabled = nextJoyModeEnabled;
  renderJoyModeButton();
});

spin2Btn.addEventListener("click", () => {
  if (!joyModeEnabled) {
    return;
  }
  sendWsMessage({ type: "spin", value: 2 });
});

spin1Btn.addEventListener("click", () => {
  if (!joyModeEnabled) {
    return;
  }
  sendWsMessage({ type: "spin", value: 1 });
});

spin0Btn.addEventListener("click", () => {
  if (!joyModeEnabled) {
    return;
  }
  sendWsMessage({ type: "spin", value: 0 });
});

focusBtn.addEventListener("click", () => {
  if (controllerBusy) {
    return;
  }
  requestFocusToggle();
});

focusBtn.addEventListener("pointerup", (event) => {
  if (controllerBusy) {
    return;
  }
  event.preventDefault();
  requestFocusToggle();
});

estopBtn.addEventListener("click", () => {
  if (controllerBusy) {
    return;
  }
  releaseJoystick();
  sendWsMessage({ type: "estop" });
});

window.addEventListener("beforeunload", () => {
  stopHoldResendLoop();
  releaseJoystick();
});

connectWebSocket();
renderJoyModeButton();
renderFocusButton();