import { jointTravelMetrics, placeJointInspectionCard } from "./live-calibration-inspection.js";

const degrees = (value, digits = 1) => `${value >= 0 ? "+" : ""}${value.toFixed(digits)}°`;
const jointName = (label) => label.replace(/\b(HIP|UPPER|LOWER)\b/g, (name) => name.toLowerCase());
const sourceName = (source) => source.includes("CALIBRATION DRAFT") ? "Calibration draft · model preview"
  : source.includes("DRAFT") ? "Draft limits · model preview"
    : source.includes("ROBOT") ? "Robot profile · model preview" : "Simulation · model preview";

export function createJointInspectionOverlay(root, { onFocus } = {}) {
  const detailId = `${root.id || "joint-inspection"}-details`;
  root.innerHTML = `
    <svg class="joint-screen-leader" aria-hidden="true"><path/><circle r="7"/><circle r="2"/></svg>
    <section class="joint-screen-card" aria-label="Selected joint travel inspection">
      <div class="joint-screen-heading"><strong data-field="joint">FL hip</strong><span class="joint-screen-state" data-field="status">Within range</span></div>
      <p class="joint-screen-source" data-field="source">Calibration draft · model preview</p>
      <div class="joint-screen-pose"><span>Model angle</span><strong data-field="pose">+0.0°</strong></div>
      <div class="joint-screen-travel" role="img" aria-label="Configured joint limits and model pose">
        <div class="joint-screen-track"><i class="joint-screen-allowed"></i><i class="joint-screen-neutral"></i><b class="joint-screen-current"></b></div>
        <div class="joint-screen-travel-labels"><span data-field="min">Min −30°</span><span>0° neutral</span><span data-field="max">Max +30°</span></div>
      </div>
      <div class="joint-screen-actions">
        <button type="button" class="joint-screen-collapse" aria-label="Expand joint details" aria-controls="${detailId}" aria-expanded="false">Details</button>
        <button type="button" class="joint-screen-focus">Focus joint</button>
      </div>
      <div class="joint-screen-detail" id="${detailId}" hidden>
        <dl>
          <div><dt>Servo command (est.)</dt><dd data-field="servo">129.9°</dd></div>
          <div data-target-row><dt>Target angle</dt><dd data-field="target">+0.0°</dd></div>
          <div><dt>To minimum</dt><dd data-field="minMargin">30.0°</dd></div>
          <div><dt>To maximum</dt><dd data-field="maxMargin">30.0°</dd></div>
          <div><dt>Output channel</dt><dd data-field="channel">0</dd></div>
          <div><dt>Local axis</dt><dd data-field="axis">X</dd></div>
        </dl>
        <p>Joint position is shown from the model, not a measured encoder.</p>
      </div>
    </section>`;
  const card = root.querySelector(".joint-screen-card");
  const fields = Object.fromEntries([...root.querySelectorAll("[data-field]")]
    .map((element) => [element.dataset.field, element]));
  const allowed = root.querySelector(".joint-screen-allowed");
  const current = root.querySelector(".joint-screen-current");
  const targetRow = root.querySelector("[data-target-row]");
  const range = root.querySelector(".joint-screen-travel");
  const collapse = root.querySelector(".joint-screen-collapse");
  const focus = root.querySelector(".joint-screen-focus");
  const details = root.querySelector(".joint-screen-detail");
  const leader = root.querySelector(".joint-screen-leader");
  const line = leader.querySelector("path");
  const dots = leader.querySelectorAll("circle");
  let side = "left";
  let lastKey = "";
  let selected = "";
  let expanded = false;
  collapse.addEventListener("click", () => {
    expanded = !expanded;
    details.hidden = !expanded;
    collapse.setAttribute("aria-expanded", String(expanded));
    collapse.setAttribute("aria-label", `${expanded ? "Collapse" : "Expand"} joint details`);
    collapse.textContent = expanded ? "Hide details" : "Details";
  });
  focus.addEventListener("click", () => onFocus?.());
  return {
    hide() { root.hidden = true; },
    render(state, viewport, anchor, obstacles = []) {
      const metrics = jointTravelMetrics(state.poseDeg, state.minimumDeg, state.maximumDeg, state.travelDeg);
      if (!metrics) { root.hidden = true; return; }
      root.hidden = false;
      if (selected !== state.label) { selected = state.label; side = "left"; }
      root.style.left = `${viewport.left}px`;
      root.style.top = `${viewport.top}px`;
      root.style.width = `${viewport.width}px`;
      root.style.height = `${viewport.height}px`;
      root.dataset.compact = String(viewport.width < 600 || viewport.height < 360);
      const key = [state.label, state.source, state.poseDeg.toFixed(1), state.servoDeg.toFixed(1),
        state.minimumDeg, state.maximumDeg, state.targetDeg?.toFixed(1), state.channel,
        state.axis, state.canFocus, anchor.inView, state.modelWarning].join(":");
      if (lastKey !== key) {
        lastKey = key;
        card.dataset.state = state.modelWarning && metrics.state === "clear" ? "near" : metrics.state;
        fields.source.textContent = sourceName(state.source);
        fields.joint.textContent = jointName(state.label);
        fields.pose.textContent = degrees(state.poseDeg);
        fields.servo.textContent = `${state.servoDeg.toFixed(1)}°`;
        fields.min.textContent = `Min ${degrees(state.minimumDeg, 0)}`;
        fields.max.textContent = `Max ${degrees(state.maximumDeg, 0)}`;
        fields.minMargin.textContent = metrics.minimumMargin < -0.05
          ? `${Math.abs(metrics.minimumMargin).toFixed(1)}° over` : `${Math.max(0, metrics.minimumMargin).toFixed(1)}°`;
        fields.maxMargin.textContent = metrics.maximumMargin < -0.05
          ? `${Math.abs(metrics.maximumMargin).toFixed(1)}° over` : `${Math.max(0, metrics.maximumMargin).toFixed(1)}°`;
        const stop = metrics.nearestStop === "MIN" ? "minimum" : "maximum";
        fields.status.textContent = !anchor.inView ? "Out of view" : state.modelWarning ? "Check clearance"
          : metrics.state === "outside" ? `Beyond ${stop}`
            : metrics.state === "stop" ? `At ${stop}`
              : metrics.state === "near" ? `Near ${stop}` : "Within range";
        fields.status.title = state.modelWarning || "Checks the selected model angle against the configured travel.";
        fields.channel.textContent = String(state.channel);
        fields.axis.textContent = state.axis;
        focus.hidden = !state.canFocus;
        range.setAttribute("aria-label", `Allowed ${degrees(state.minimumDeg)} to ${degrees(state.maximumDeg)}. Model pose ${degrees(state.poseDeg)}. ${fields.status.textContent}.`);
        allowed.style.left = `${metrics.minimumPercent}%`;
        allowed.style.width = `${metrics.maximumPercent - metrics.minimumPercent}%`;
        current.style.left = `${metrics.posePercent}%`;
        targetRow.hidden = !Number.isFinite(state.targetDeg);
        if (!targetRow.hidden) fields.target.textContent = degrees(state.targetDeg);
      }
      const bounds = card.getBoundingClientRect();
      const position = placeJointInspectionCard(anchor, viewport, bounds, side, obstacles);
      side = position.side;
      card.style.transform = `translate(${position.x}px, ${position.y}px)`;
      leader.setAttribute("viewBox", `0 0 ${viewport.width} ${viewport.height}`);
      leader.style.display = anchor.inView ? "" : "none";
      line.setAttribute("d", `M ${anchor.x} ${anchor.y} L ${position.edgeX} ${position.edgeY}`);
      dots.forEach((dot) => { dot.setAttribute("cx", anchor.x); dot.setAttribute("cy", anchor.y); });
    },
  };
}
