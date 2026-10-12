export function desktopWindowTitle(appName, version) {
  const name = String(appName || "").trim();
  const normalizedVersion = String(version || "").trim().replace(/^v/i, "");
  if (!normalizedVersion) return name;
  return `${name} — v${normalizedVersion}`;
}
