const buttonResults = new WeakMap();

export function setStudioActionFeedback(output, message, state = "info") {
  if (!output) return;
  if (output.textContent !== message) output.textContent = message;
  if (output.dataset.feedback !== state) output.dataset.feedback = state;
  output.classList.add("studio-action-feedback");
  output.setAttribute("role", "status");
  output.setAttribute("aria-live", "polite");
}

// Called only after the storage operation or acknowledgement has succeeded.
export function showStudioButtonResult(button, label, state = "success") {
  if (!button) return;
  const previous = buttonResults.get(button);
  if (previous) clearTimeout(previous.timer);
  const original = previous?.original ?? button.textContent;
  const result = { original, timer: null };
  buttonResults.set(button, result);
  button.textContent = label;
  button.dataset.feedback = state;
  result.timer = setTimeout(() => {
    if (buttonResults.get(button) !== result) return;
    if (button.textContent === label && button.getAttribute("aria-busy") !== "true") {
      button.textContent = original;
    }
    delete button.dataset.feedback;
    buttonResults.delete(button);
  }, 2800);
}
