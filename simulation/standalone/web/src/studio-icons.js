export const CLOSE_ICON = '<svg viewBox="0 0 20 20" aria-hidden="true" focusable="false"><path d="M5 5l10 10M15 5L5 15" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/></svg>';

export function setCloseIcon(button) {
  button.classList.add("studio-icon-button");
  button.innerHTML = CLOSE_ICON;
}
