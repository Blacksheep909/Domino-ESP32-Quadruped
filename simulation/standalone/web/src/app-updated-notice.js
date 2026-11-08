// One-time "Studio was updated" notice on the first launch after an update.
// The desktop updater reports the running version; the last version this PC
// launched is remembered in localStorage. A fresh install stays quiet.

const RELEASES_URL = "https://github.com/Blacksheep909/Domino-ESP32-Quadruped/releases/tag/v";
export const LAST_SEEN_VERSION_KEY = "domino.studio.lastSeenVersion";
const VISIBLE_MS = 15_000;

function parts(version) {
  const match = String(version || "").match(/^(\d+)\.(\d+)\.(\d+)/);
  return match ? match.slice(1).map(Number) : null;
}

export function compareVersions(a, b) {
  const pa = parts(a);
  const pb = parts(b);
  if (!pa || !pb) return 0;
  for (let i = 0; i < 3; i += 1) {
    if (pa[i] !== pb[i]) return pa[i] > pb[i] ? 1 : -1;
  }
  return 0;
}

// Pure description of the notice: shown only when this launch runs a newer
// version than the last one seen on this PC.
export function updatedNoticeView(currentVersion, lastSeenVersion) {
  const current = String(currentVersion || "");
  if (!parts(current) || !parts(lastSeenVersion) || compareVersions(current, lastSeenVersion) <= 0) {
    return { visible: false };
  }
  return {
    visible: true,
    text: `Studio updated to ${current}`,
    detail: `Previously ${lastSeenVersion}. If this release includes new firmware, flash it from FIRMWARE.`,
    notesUrl: `${RELEASES_URL}${current}`,
  };
}

function storage() {
  try {
    return globalThis.localStorage ?? null;
  } catch {
    return null;
  }
}

export async function initUpdatedNotice(updates, { root = document.body } = {}) {
  if (!updates || typeof updates.getState !== "function") return null;
  let current = "";
  try {
    current = String((await updates.getState())?.currentVersion || "");
  } catch {
    return null;
  }
  if (!parts(current)) return null;
  const store = storage();
  let lastSeen = null;
  try {
    lastSeen = store?.getItem(LAST_SEEN_VERSION_KEY) ?? null;
    store?.setItem(LAST_SEEN_VERSION_KEY, current);
  } catch {
    // Storage unavailable: show nothing rather than on every launch.
    return null;
  }
  const view = updatedNoticeView(current, lastSeen);
  if (!view.visible) return null;

  const banner = document.createElement("div");
  banner.className = "app-update-banner app-updated-notice";
  banner.dataset.tone = "success";
  banner.setAttribute("role", "status");
  banner.setAttribute("aria-live", "polite");
  banner.innerHTML = `
    <div class="app-update-copy"><strong></strong><span></span></div>
    <div class="app-update-actions">
      <a class="app-update-notes" target="_blank" rel="noopener">WHAT'S NEW</a>
      <button type="button" class="app-update-dismiss">OK</button>
    </div>`;
  banner.querySelector(".app-update-copy strong").textContent = view.text;
  banner.querySelector(".app-update-copy span").textContent = view.detail;
  banner.querySelector(".app-update-notes").href = view.notesUrl;
  const close = () => banner.remove();
  banner.querySelector(".app-update-dismiss").addEventListener("click", close);
  setTimeout(close, VISIBLE_MS);
  root.append(banner);
  return banner;
}
