export const RADIO_INPUT_TIMEOUT_MS = 250;
export const CLIENT_CONTROL_TIMEOUT_MS = 250;

export function radioInputIsFresh(input, now = Date.now()) {
  return Boolean(
    input?.connected &&
    Array.isArray(input.channels) &&
    now - Number(input.updatedAt) < RADIO_INPUT_TIMEOUT_MS,
  );
}

export function releaseClientControl(socket) {
  socket.controlActive = false;
  socket.controlMode = null;
  socket.manualOverride = false;
  socket.lastControlAt = 0;
}

export function clientControlIsFresh(socket, now = Date.now()) {
  return Boolean(
    socket?.controlActive === true &&
    now - Number(socket.lastControlAt) < CLIENT_CONTROL_TIMEOUT_MS,
  );
}

export function hasActiveClient(clients, predicate, now = Date.now()) {
  return [...clients].some(
    (socket) => clientControlIsFresh(socket, now) && predicate(socket),
  );
}
