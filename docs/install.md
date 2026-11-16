# Installing a release

Each [release](https://github.com/Sparkz400/KinematicsSimulation/releases) has a Windows zip and MSI, a Linux tarball and .deb, a macOS tarball (one universal build for Apple silicon and Intel), FMUs and `SHA256SUMS`. Check a download with `sha256sum -c SHA256SUMS --ignore-missing`, and that GitHub's release workflow built it with `gh attestation verify FILE --repo Sparkz400/KinematicsSimulation`.

Nothing is code-signed. Windows SmartScreen may warn ("More info", "Run anyway"). On macOS the build is neither signed nor notarised, so Gatekeeper refuses to open it when it came through a browser; after unpacking, take the quarantine flag off the folder and start the programs from a terminal:

```sh
tar -xzf kinematics-simulation-<version>-macos-universal.tar.gz
xattr -dr com.apple.quarantine kinematics-simulation-<version>-macos-universal
kinematics-simulation-<version>-macos-universal/bin/sim-app
```

(Or, after the first refusal, System Settings → Privacy & Security → "Open Anyway" for that program.) The macOS build is made on GitHub's macOS runner and has not been run on a Mac by the authors.

