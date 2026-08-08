const canvas = document.querySelector("#robot");
const context = canvas.getContext("2d");
const servoContainer = document.querySelector("#servos");

const legs = [
  { name: "FL", hip: 0, upper: 1, lower: 2, upperTrim: 0, lowerTrim: 10, upperDir: -1, lowerDir: -1 },
  { name: "FR", hip: 3, upper: 4, lower: 15, upperTrim: -20, lowerTrim: -38, upperDir: 1, lowerDir: 1 },
  { name: "BL", hip: 14, upper: 7, lower: 8, upperTrim: -20, lowerTrim: -5, upperDir: -1, lowerDir: -1 },
  { name: "BR", hip: 9, upper: 10, lower: 11, upperTrim: -20, lowerTrim: -30, upperDir: 1, lowerDir: 1 },
];

const activeChannels = [
  ["FL H", 0], ["FL U", 1], ["FL L", 2],
  ["FR H", 3], ["FR U", 4], ["FR L", 15],
  ["BL H", 14], ["BL U", 7], ["BL L", 8],
  ["BR H", 9], ["BR U", 10], ["BR L", 11],
];

const servoRows = activeChannels.map(([label, channel]) => {
  const row = document.createElement("div");
  row.className = "servo-row";
  row.innerHTML = `
    <span>${label}</span>
    <span class="servo-track"><span class="servo-fill"></span></span>
    <span class="servo-value">0.0°</span>
  `;
  servoContainer.appendChild(row);
  return { channel, fill: row.querySelector(".servo-fill"), value: row.querySelector(".servo-value") };
});

let state = null;
let lastReceived = 0;

function resizeCanvas() {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function point(x, y) {
  return { x, y };
}

function line(a, b, color, width) {
  context.beginPath();
  context.moveTo(a.x, a.y);
  context.lineTo(b.x, b.y);
  context.strokeStyle = color;
  context.lineWidth = width;
  context.lineCap = "round";
  context.stroke();
}

function joint(position, radius, color, outline = "#f7f9fa") {
  context.beginPath();
  context.arc(position.x, position.y, radius, 0, Math.PI * 2);
  context.fillStyle = color;
  context.fill();
  context.lineWidth = 2;
  context.strokeStyle = outline;
  context.stroke();
}

function decodeLeg(leg, angles) {
  const theta2 = (angles[leg.upper] - 135 - leg.upperTrim) / leg.upperDir;
  const theta3 = (angles[leg.lower] - 135 - leg.lowerTrim) / leg.lowerDir;
  return {
    upperRad: theta2 * Math.PI / 180,
    lowerRad: theta3 * Math.PI / 180,
  };
}

function drawRobot(currentState) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  context.clearRect(0, 0, width, height);

  const scale = Math.min(width / 850, height / 540);
  const center = point(width * 0.5, height * 0.36);
  const bodyLength = 330 * scale;
  const bodyDepth = 116 * scale;
  const bodySkew = 42 * scale;
  const body = [
    point(center.x - bodyLength / 2, center.y - bodyDepth / 2),
    point(center.x + bodyLength / 2, center.y - bodyDepth / 2 + bodySkew),
    point(center.x + bodyLength / 2, center.y + bodyDepth / 2 + bodySkew),
    point(center.x - bodyLength / 2, center.y + bodyDepth / 2),
  ];

  context.beginPath();
  context.moveTo(body[0].x, body[0].y);
  body.slice(1).forEach((p) => context.lineTo(p.x, p.y));
  context.closePath();
  context.fillStyle = "#303b43";
  context.fill();
  context.lineWidth = 8 * scale;
  context.strokeStyle = "#151d22";
  context.stroke();

  line(point(body[0].x + 22 * scale, body[0].y + 12 * scale),
       point(body[2].x - 22 * scale, body[2].y - 12 * scale), "#7b8992", 4 * scale);
  line(point(body[3].x + 20 * scale, body[3].y - 10 * scale),
       point(body[1].x - 20 * scale, body[1].y + 10 * scale), "#7b8992", 4 * scale);

  const hipPoints = [body[0], body[1], body[3], body[2]];
  const drawOrder = [0, 1, 2, 3];
  drawOrder.forEach((index) => {
    const leg = legs[index];
    const decoded = decodeLeg(leg, currentState.servo_angle_deg);
    const hip = hipPoints[index];
    const side = index % 2 === 0 ? -1 : 1;
    const rear = index >= 2;
    const upperLength = 145 * scale;
    const lowerLength = 142 * scale;
    const planeShift = side * 0.12 * (currentState.servo_angle_deg[leg.hip] - 110) * scale;
    const direction = rear ? -1 : 1;

    const knee = point(
      hip.x + direction * Math.sin(decoded.upperRad) * upperLength + planeShift,
      hip.y + Math.cos(decoded.upperRad) * upperLength,
    );
    const foot = point(
      knee.x + direction * Math.sin(decoded.lowerRad) * lowerLength + planeShift * 0.2,
      knee.y + Math.cos(decoded.lowerRad) * lowerLength,
    );
    const pinOffset = 9 * scale;
    const hipPin = point(hip.x + pinOffset, hip.y - pinOffset);
    const kneePin = point(knee.x + pinOffset, knee.y - pinOffset);

    line(hip, knee, "#11191e", 8 * scale);
    line(knee, foot, "#11191e", 7 * scale);
    line(hipPin, kneePin, "#53616a", 4 * scale);
    line(hip, hipPin, "#53616a", 3 * scale);
    line(knee, kneePin, "#53616a", 3 * scale);
    joint(hip, 8 * scale, "#e36b2c");
    joint(knee, 7 * scale, "#e36b2c");
    joint(hipPin, 5 * scale, "#65747e");
    joint(kneePin, 5 * scale, "#65747e");
    joint(foot, 11 * scale, "#12191e");

    context.fillStyle = "#53616a";
    context.font = `600 ${11 * scale}px "Cascadia Mono", Consolas, monospace`;
    context.fillText(leg.name, hip.x + 11 * scale, hip.y - 12 * scale);
  });
}

function updateTelemetry(currentState) {
  document.querySelector("#mode").textContent = currentState.mode;
  document.querySelector("#link").textContent = currentState.link_alive ? "LINK OK" : "LINK LOST";
  document.querySelector("#clock").textContent = `${(currentState.elapsed_ms / 1000).toFixed(2)} s`;
  document.querySelector("#target-z").textContent = `${currentState.target_z_mm.toFixed(1)} mm`;
  document.querySelector("#pose-z").textContent = `${currentState.pose_z_mm.toFixed(1)} mm`;
  document.querySelector("#frames").textContent = currentState.accepted_frames;
  document.querySelector("#sa").textContent = `${currentState.channels_us[4]} us`;
  document.querySelector("#sb").textContent = `${currentState.channels_us[5]} us`;
  document.querySelector("#sc").textContent = `${currentState.channels_us[6]} us`;
  document.querySelector("#sd").textContent = `${currentState.channels_us[7]} us`;

  servoRows.forEach(({ channel, fill, value }) => {
    const angle = currentState.servo_angle_deg[channel];
    fill.style.width = `${Math.max(0, Math.min(100, angle / 2.7))}%`;
    value.textContent = `${angle.toFixed(1)}°`;
  });
}

async function pollState() {
  try {
    const candidates = await Promise.all([0, 1].map(async (slot) => {
      try {
        const response = await fetch(`runtime/state.json.${slot}?t=${Date.now()}`, {
          cache: "no-store",
        });
        return response.ok ? await response.json() : null;
      } catch {
        return null;
      }
    }));
    state = candidates
      .filter(Boolean)
      .sort((a, b) => Number(b.elapsed_ms) - Number(a.elapsed_ms))[0];
    if (!state) throw new Error("Firmware state unavailable");
    lastReceived = Date.now();
    document.querySelector("#connection").textContent = "Live firmware output";
    updateTelemetry(state);
  } catch {
    if (Date.now() - lastReceived > 1000) {
      document.querySelector("#connection").textContent = "Waiting for simulator";
    }
  }
}

function render() {
  if (state) {
    drawRobot(state);
  }
  requestAnimationFrame(render);
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
setInterval(pollState, 80);
pollState();
render();
