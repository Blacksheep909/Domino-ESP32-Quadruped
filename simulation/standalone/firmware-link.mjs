// The uploader owns the serial port until its child has fully stopped.
// Keep this on the server so closing the upload dialog cannot skip recovery.
export class FirmwareLinkCoordinator {
  constructor({ firmware, companion, pairedAdapter = () => null, notify = () => {} }) {
    Object.assign(this, { firmware, companion, pairedAdapter, notify });
    this.busy = false;
    this.restore = null;
  }

  async startUpload(options) {
    if (this.busy) throw new Error("A firmware upload is already in progress.");
    this.firmware.validateUpload(options);
    const previous = this.companion.status();
    const paired = this.pairedAdapter();
    if (paired?.robotState === "armed") throw new Error("Disarm the robot before uploading firmware.");
    if (paired?.calibrationPending) {
      throw new Error("Wait for the calibration request and its save read-back to finish before uploading firmware.");
    }
    const port = options.port || (previous.transport === "usb" ? previous.endpoint : "");
    this.busy = true;
    this.restore = previous.running ? { ...previous, adapterId: paired?.adapterId || "" } : null;
    this.notify({ busy: true, adapterId: paired?.adapterId || "", stage: "Releasing robot connection" });
    try {
      await this.companion.stop();
      return this.firmware.startUpload({ ...options, port });
    } catch (error) {
      await this.finish();
      throw error;
    }
  }

  async finish(job) {
    if (!this.busy || (job && job.type !== "upload")) return;
    const restore = this.restore;
    this.restore = null;
    let error = "";
    try {
      if (restore) await this.companion.start(restore.transport, restore.endpoint);
    } catch (failure) { error = failure.message || "The robot connection could not restart."; }
    this.busy = false;
    this.notify({ busy: false, adapterId: restore?.adapterId || "", stage: error || "Robot connection released", error });
  }
}
