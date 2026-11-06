// electron-builder afterPack hook.
//
// The macOS build is not signed with an Apple Developer ID (identity: null).
// Apple-silicon Macs refuse to launch arm64 code with no signature at all, so
// give the finished .app an ad-hoc signature before it is put into the DMG and
// zip. Users still open it the first time with right-click > Open (Gatekeeper),
// but it launches instead of being reported as damaged.
const { execFileSync } = require("node:child_process");
const path = require("node:path");

exports.default = async function afterPack(context) {
  if (context.electronPlatformName !== "darwin") return;
  const appName = `${context.packager.appInfo.productFilename}.app`;
  const appPath = path.join(context.appOutDir, appName);
  console.log(`  • ad-hoc signing ${appPath}`);
  execFileSync("codesign", ["--force", "--deep", "--sign", "-", appPath], { stdio: "inherit" });
  execFileSync("codesign", ["--verify", "--deep", "--strict", appPath], { stdio: "inherit" });
};
