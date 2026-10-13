// A single transient notification stays separate from page layout and dialogs.
import { setCloseIcon } from "./studio-icons.js";
// Popover puts it above native dialogs without moving keyboard focus.
export function createStudioNotifications(region) {
  let timeout = null;
  let active = null;
  let remaining = 0;
  let expiresAt = 0;
  let returnFocus = null;
  const clock = region.ownerDocument.defaultView;

  function dismiss() {
    const restore = region.contains(region.ownerDocument.activeElement);
    clock.clearTimeout(timeout);
    timeout = null;
    active = null;
    region.replaceChildren();
    if (region.matches(":popover-open")) region.hidePopover();
    if (restore && returnFocus?.isConnected) returnFocus.focus({ preventScroll: true });
  }

  function schedule(duration) {
    clock.clearTimeout(timeout);
    remaining = duration;
    expiresAt = Date.now() + duration;
    timeout = duration > 0 ? clock.setTimeout(dismiss, duration) : null;
  }

  region.addEventListener("pointerenter", () => {
    if (!timeout) return;
    remaining = Math.max(1, expiresAt - Date.now());
    clock.clearTimeout(timeout);
    timeout = null;
  });
  region.addEventListener("pointerleave", () => {
    if (active && remaining > 0 && !region.contains(region.ownerDocument.activeElement)) schedule(remaining);
  });
  region.addEventListener("focusin", () => {
    if (!timeout) return;
    remaining = Math.max(1, expiresAt - Date.now());
    clock.clearTimeout(timeout);
    timeout = null;
  });
  region.addEventListener("focusout", () => {
    clock.setTimeout(() => {
      if (active && remaining > 0 && !region.contains(region.ownerDocument.activeElement)) schedule(remaining);
    }, 0);
  });

  return {
    show(message, tone = "info", durationMs = 3600, key = "") {
      if (!message) return;
      if (active?.message === message && active?.tone === tone && active?.key === key) return;
      clock.clearTimeout(timeout);
      if (!region.contains(region.ownerDocument.activeElement)) returnFocus = region.ownerDocument.activeElement;
      else if (returnFocus?.isConnected) returnFocus.focus({ preventScroll: true });
      const toast = region.ownerDocument.createElement("div");
      toast.className = "app-toast is-visible";
      toast.dataset.tone = tone;
      const content = region.ownerDocument.createElement("span");
      content.className = "app-toast-message";
      content.setAttribute("role", "status");
      content.setAttribute("aria-live", "polite");
      content.setAttribute("aria-atomic", "true");
      content.textContent = message;
      const close = region.ownerDocument.createElement("button");
      close.type = "button";
      close.className = "app-toast-close";
      setCloseIcon(close);
      close.setAttribute("aria-label", "Dismiss notification");
      close.addEventListener("click", dismiss);
      toast.append(content, close);
      region.replaceChildren(toast);
      active = { message, tone, key };
      if (!region.matches(":popover-open")) region.showPopover();
      schedule(durationMs);
    },
    dismiss,
  };
}
