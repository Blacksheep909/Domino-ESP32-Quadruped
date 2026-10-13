import { CAMERA_CONTROL_PRESETS, sanitizeCameraPreset } from "./camera-control-presets.js";
import { findStudioGuides, STUDIO_HELP_GUIDES, STUDIO_KEYBINDS } from "./studio-help-content.js";
import { setStudioActionFeedback } from "./studio-action-feedback.js";

const element = (tag, className, text) => {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
};

function instruction(text) {
  const line = element("li");
  for (const part of text.split(/(\*\*[^*]+\*\*)/)) {
    line.append(part.startsWith("**") ? element("strong", "", part.slice(2, -2)) : document.createTextNode(part));
  }
  return line;
}

export function createStudioHelp({ getContext, getCameraPreset, onCameraChange, onNavigate }) {
  const dialog = document.querySelector("#shortcuts-dialog");
  const search = document.querySelector("#studio-help-search");
  const navigation = document.querySelector("#studio-help-navigation");
  const article = document.querySelector("#studio-help-article");
  let selectedGuide = "start";
  let opener = null;
  let section = "guides";

  function menuLink(target, label) {
    const button = element("button", "help-open-target", label);
    button.type = "button";
    button.addEventListener("click", () => { close(); onNavigate(target); });
    return button;
  }

  function renderNavigation() {
    const matches = findStudioGuides(search.value);
    navigation.replaceChildren();
    const chooser = document.querySelector("#studio-help-guide-select");
    chooser.replaceChildren();
    for (const entry of matches) {
      const option = element("option", "", `${entry.group} / ${entry.title}`);
      option.value = entry.id;
      chooser.append(option);
    }
    chooser.value = selectedGuide;
    chooser.disabled = !matches.length;
    for (const group of [...new Set(matches.map((entry) => entry.group))]) {
      const heading = element("h3", "help-nav-group", group);
      navigation.append(heading);
      for (const entry of matches.filter((candidate) => candidate.group === group)) {
        const button = element("button", "help-guide-link", entry.title);
        button.type = "button";
        button.setAttribute("aria-current", entry.id === selectedGuide ? "page" : "false");
        button.addEventListener("click", () => {
          selectedGuide = entry.id;
          renderNavigation();
          renderArticle();
          article.focus();
        });
        navigation.append(button);
      }
    }
    if (!matches.length) navigation.append(element("p", "help-no-results", "No matching guide. Try a tab name, JSON, camera or connection."));
    document.querySelector("#studio-help-search-count").textContent = `${matches.length} guides`;
  }

  function renderArticle() {
    const entry = STUDIO_HELP_GUIDES.find(({ id }) => id === selectedGuide) || STUDIO_HELP_GUIDES[0];
    article.replaceChildren();
    const header = element("div", "help-article-heading");
    const text = element("div");
    text.append(element("span", "help-eyebrow", entry.group), element("h3", "", entry.title), element("p", "", entry.summary));
    header.append(text, menuLink(entry.target, entry.openLabel));
    article.append(header);
    for (const part of entry.sections) {
      const details = element("section", "help-guide-section");
      const heading = element("div", "help-section-heading");
      heading.append(element("h4", "", part.title));
      if (part.target) heading.append(menuLink(part.target, part.openLabel));
      const steps = element("ol", "help-steps");
      for (const step of part.steps) steps.append(instruction(step));
      details.append(heading, steps);
      article.append(details);
    }
    article.scrollTop = 0;
  }

  function selectSection(next) {
    section = next;
    document.querySelectorAll("[data-help-tab]").forEach((button) => {
      const active = button.dataset.helpTab === next;
      button.setAttribute("aria-selected", String(active));
      button.tabIndex = active ? 0 : -1;
    });
    document.querySelectorAll("[data-help-panel]").forEach((panel) => {
      panel.hidden = panel.dataset.helpPanel !== next;
    });
  }

  function renderCamera() {
    const select = document.querySelector("#studio-camera-preset");
    select.value = sanitizeCameraPreset(getCameraPreset());
    const preset = CAMERA_CONTROL_PRESETS.find(({ id }) => id === select.value);
    const bindings = document.querySelector("#studio-camera-bindings");
    bindings.replaceChildren();
    for (const [label, value] of [["ORBIT", preset.orbit], ["PAN", preset.pan], ["ZOOM", preset.zoom]]) {
      const row = element("div", "help-camera-binding");
      row.append(element("dt", "", label), element("dd", "", value));
      bindings.append(row);
    }
    const source = document.querySelector("#studio-camera-source");
    source.hidden = !preset.source;
    source.href = preset.source || "#";
    source.textContent = `${preset.label} navigation reference`;
    document.querySelector("#studio-camera-selected").textContent = `${preset.label} controls are active in every 3D viewport.`;
  }

  function open(from = null, guideId = null) {
    opener = from || document.activeElement;
    selectedGuide = guideId || getContext() || "start";
    search.value = "";
    renderNavigation();
    renderArticle();
    renderCamera();
    selectSection("guides");
    dialog.hidden = false;
    if (!dialog.open) dialog.showModal();
    search.focus();
  }

  function close() {
    if (dialog.open) dialog.close();
    dialog.hidden = true;
    opener?.focus();
    opener = null;
  }

  search.addEventListener("input", renderNavigation);
  document.querySelector("#studio-help-guide-select").addEventListener("change", (event) => {
    selectedGuide = event.currentTarget.value;
    renderNavigation();
    renderArticle();
    article.focus();
  });
  document.querySelector("#shortcuts-open").addEventListener("click", (event) => open(event.currentTarget));
  for (const id of ["shortcuts-close", "shortcuts-done"]) document.querySelector(`#${id}`).addEventListener("click", close);
  dialog.addEventListener("click", (event) => { if (event.target === dialog) close(); });
  dialog.addEventListener("cancel", (event) => {
    event.preventDefault();
    close();
  });
  const tabs = [...document.querySelectorAll("[data-help-tab]")];
  tabs.forEach((button, index) => {
    button.addEventListener("click", () => selectSection(button.dataset.helpTab));
    button.addEventListener("keydown", (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      const next = event.key === "Home" ? 0 : event.key === "End" ? tabs.length - 1
        : (index + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) % tabs.length;
      selectSection(tabs[next].dataset.helpTab);
      tabs[next].focus();
    });
  });
  const cameraSelect = document.querySelector("#studio-camera-preset");
  for (const preset of CAMERA_CONTROL_PRESETS) {
    const option = element("option", "", preset.label);
    option.value = preset.id;
    cameraSelect.append(option);
  }
  cameraSelect.addEventListener("change", () => {
    const saved = onCameraChange(sanitizeCameraPreset(cameraSelect.value));
    renderCamera();
    const preset = CAMERA_CONTROL_PRESETS.find(({ id }) => id === cameraSelect.value);
    setStudioActionFeedback(document.querySelector("#studio-camera-selected"), saved === false
      ? `${preset.label} controls are active for this session. Browser storage is unavailable, so the preset could not be saved.`
      : `${preset.label} controls applied and saved on this PC.`, saved === false ? "warning" : "success");
  });
  const keybinds = document.querySelector("#studio-keybinds-body");
  for (const [keys, action, context] of STUDIO_KEYBINDS) {
    const row = element("tr");
    const key = element("td");
    key.append(element("kbd", "", keys));
    row.append(key, element("td", "", action), element("td", "", context));
    keybinds.append(row);
  }

  return { open, close, isOpen: () => !dialog.hidden, selectedSection: () => section };
}
