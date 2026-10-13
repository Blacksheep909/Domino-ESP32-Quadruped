import { spawn } from "node:child_process";
import { setTimeout as delay } from "node:timers/promises";

// Wait for close (including stdio), not just exit. Windows may take a little
// longer to release a serial handle after the owning process has closed.
export async function closeCompanionProcess(child, {
  command = null, timeoutMs = 2500, releaseDelayMs = process.platform === "win32" ? 250 : 0,
} = {}) {
  if (!child || ((child.exitCode !== null || child.signalCode) && child.stdio?.every((stream) => !stream || stream.destroyed))) return;
  await new Promise((resolve, reject) => {
    let forceTimer;
    const finish = () => {
      clearTimeout(timer);
      clearTimeout(forceTimer);
      resolve();
    };
    child.once("close", finish);
    const timer = setTimeout(() => {
      // Only this owned child and its serial helper are terminated.
      if (process.platform === "win32" && child.pid) {
        const killer = spawn("taskkill.exe", ["/PID", String(child.pid), "/T", "/F"], { windowsHide: true, stdio: "ignore" });
        killer.on("error", () => child.kill());
      } else child.kill("SIGKILL");
      forceTimer = setTimeout(() => {
        child.off("close", finish);
        reject(new Error("The robot companion did not close. Upload has been blocked because the port may still be busy."));
      }, 2000);
    }, timeoutMs);
    child.stdin?.on("error", () => {}); // A closing pipe must not crash Studio.
    if (child.stdin?.writable) child.stdin.end(command ? `${JSON.stringify(command)}\n` : undefined);
    else child.kill("SIGTERM");
  });
  if (releaseDelayMs) await delay(releaseDelayMs);
}
