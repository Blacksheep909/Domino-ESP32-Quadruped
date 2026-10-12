import { jointTravelMetrics, placeJointInspectionCard } from "./live-calibration-inspection.js";

const degrees = (value, digits = 1) => `${value >= 0 ? "+" : ""}${value.toFixed(digits)}°`;

export function createJointInspectionOverlay(root, { onFocus } = {}) {
  root.innerHTML = `
    <svg class="joint-screen-leader" aria-hidden="true"><path/><circle r="7"/><circle r="2"/></svg>
    <section class="joint-screen-card" aria-label="Selected joint travel inspection">
      <div class="joint-screen-heading"><div><span data-field="source">DRAFT LIMITS</span><strong data-field="joint">FL HIP</strong></div><button type="button" class="joint-screen-collapse" aria-label="Collapse joint details" aria-expanded="true">−</button></div>
      <div class="joint-screen-pose"><div><span>CAD POSE</span><strong data-field="pose">+0.0°</strong></div><div><span>SERVO EST.</span><strong data-field="servo">129.9°</strong></div></div>
        <div class="joint-screen-travel" role="img" aria-label="Configured joint limits and model pose">
          <div class="joint-screen-travel-labels"><strong data-field="min">−30°</strong><span>0° NEUTRAL</span><strong data-field="max">+30°</strong></div>
          <div class="joint-screen-track"><i class="joint-screen-allowed"></i><i class="joint-screen-neutral"></i><i class="joint-screen-target"></i><b class="joint-screen-current"></b></div>
          <div class="joint-screen-travel-key"><span><i class="allowed-key"></i>ALLOWED</span><span><i class="pose-key"></i>POSE</span><span data-field="target">TARGET +0.0°</span></div>
        </div>
        <div class="joint-screen-state"><i></i><strong data-field="status">WITHIN LIMITS</strong></div>
      <div class="joint-screen-detail">
        <div class="joint-screen-margins"><div><span>TO MIN STOP</span><strong data-field="minMargin">30.0°</strong></div><div><span>TO MAX STOP</span><strong data-field="maxMargin">30.0°</strong></div></div>
        <div class="joint-screen-footer"><span data-field="channel">CH 0 · AXIS X</span><button type="button" class="joint-screen-focus">FOCUS JOINT</button></div>
      </div>
    </section>`;
  const card = root.querySelector(".joint-screen-card");
  const fields = Object.fromEntries([...root.querySelectorAll("[data-field]")]
    .map((element) => [element.dataset.field, element]));
  const allowed = root.querySelector(".joint-screen-allowed");
  const current = root.querySelector(".joint-screen-current");
  const target = root.querySelector(".joint-screen-target");
  const range = root.querySelector(".joint-screen-travel");
  const collapse = root.querySelector(".joint-screen-collapse");
  const focus = root.querySelector(".joint-screen-focus");
  const leader = root.querySelector(".joint-screen-leader");
  const line = leader.querySelector("path");
  const dots = leader.querySelectorAll("circle");
  let side = "left";
  let lastKey = "";
  let selected = "";
  let collapsedPreference = null;
  collapse.addEventListener("click", () => {
    const collapsed = card.dataset.collapsed !== "true";
    collapsedPreference = collapsed;
    card.dataset.collapsed = String(collapsed);
    collapse.setAttribute("aria-expanded", String(!collapsed));
    collapse.setAttribute("aria-label", `${collapsed ? "Expand" : "Collapse"} joint details`);
    collapse.textContent = collapsed ? "+" : "−";
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
      const compact = viewport.width < 600 || viewport.height < 360;
      root.dataset.compact = String(compact);
      const collapsed = collapsedPreference ?? compact;
      card.dataset.collapsed = String(collapsed);
      collapse.setAttribute("aria-expanded", String(!collapsed));
      collapse.setAttribute("aria-label", `${collapsed ? "Expand" : "Collapse"} joint details`);
      collapse.textContent = collapsed ? "+" : "−";
      const key = [state.label, state.source, state.poseDeg.toFixed(1), state.servoDeg.toFixed(1),
        state.minimumDeg, state.maximumDeg, state.targetDeg?.toFixed(1), state.channel,
        state.axis, state.canFocus, anchor.inView, compact, state.modelWarning].join(":");
      if (lastKey !== key) {
        lastKey = key;
        card.dataset.state = state.modelWarning && metrics.state === "clear" ? "near" : metrics.state;
        fields.source.textContent = compact ? state.source.includes("DRAFT") ? "DRAFT" : "ROBOT PROFILE" : state.source;
        fields.source.title = state.source;
        fields.joint.textContent = state.label;
        fields.pose.textContent = degrees(state.poseDeg);
        fields.servo.textContent = `${state.servoDeg.toFixed(1)}°`;
        fields.min.textContent = `MIN ${degrees(state.minimumDeg, 0)}`;
        fields.max.textContent = `MAX ${degrees(state.maximumDeg, 0)}`;
        fields.minMargin.textContent = metrics.minimumMargin < -0.05
          ? `${Math.abs(metrics.minimumMargin).toFixed(1)}° OVER` : `${Math.max(0, metrics.minimumMargin).toFixed(1)}°`;
        fields.maxMargin.textContent = metrics.maximumMargin < -0.05
          ? `${Math.abs(metrics.maximumMargin).toFixed(1)}° OVER` : `${Math.max(0, metrics.maximumMargin).toFixed(1)}°`;
        fields.status.textContent = !anchor.inView ? "JOINT OUT OF VIEW" : state.modelWarning ? state.modelWarning
          : metrics.state === "outside" ? `OUTSIDE ${metrics.nearestStop} LIMIT`
            : metrics.state === "stop" ? `AT ${metrics.nearestStop} STOP`
              : metrics.state === "near" ? `NEAR ${metrics.nearestStop} STOP` : "WITHIN LIMITS";
        fields.channel.textContent = `CH ${state.channel} · AXIS ${state.axis}`;
        focus.hidden = !state.canFocus;
        range.setAttribute("aria-label", `Allowed ${degrees(state.minimumDeg)} to ${degrees(state.maximumDeg)}. Model pose ${degrees(state.poseDeg)}. ${fields.status.textContent}.`);
        allowed.style.left = `${metrics.minimumPercent}%`;
        allowed.style.width = `${metrics.maximumPercent - metrics.minimumPercent}%`;
        current.style.left = `${metrics.posePercent}%`;
        target.hidden = !Number.isFinite(state.targetDeg);
        fields.target.hidden = target.hidden;
        if (!target.hidden) {
          target.style.left = `${jointTravelMetrics(state.targetDeg, state.minimumDeg, state.maximumDeg, state.travelDeg).posePercent}%`;
          fields.target.textContent = `TARGET ${degrees(state.targetDeg)}`;
        }
      }
      const bounds = card.getBoundingClientRect();
      const position = placeJointInspectionCard(anchor, viewport, bounds, side, obstacles);
      side = position.side;
      card.style.transform = `translate(${position.x}px, ${position.y}px)`;
      leader.setAttribute("viewBox", `0 0 ${viewport.width} ${viewport.height}`);
      leader.hidden = !anchor.inView;
      leader.style.display = anchor.inView ? "" : "none";
      line.setAttribute("d", `M ${anchor.x} ${anchor.y} L ${position.edgeX} ${position.edgeY}`);
      dots.forEach((dot) => { dot.setAttribute("cx", anchor.x); dot.setAttribute("cy", anchor.y); });
    },
  };
}
