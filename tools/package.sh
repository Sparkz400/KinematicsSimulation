#!/usr/bin/env sh
# Builds the release binaries and packs them, unsigned, into dist/kinematics-simulation-<version>-linux-x64.tar.gz
# with README, licences, docs, the three example configs and THIRD-PARTY-LICENSES.md (see tools/package.ps1).
# On macOS: kinematics-simulation-<version>-macos-universal.tar.gz, each binary built for arm64 and x86_64 and joined
# with lipo (needs `rustup target add aarch64-apple-darwin x86_64-apple-darwin`).
# Run from anywhere: sh tools/package.sh. Honours CARGO_TARGET_DIR.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

version=$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)
target=${CARGO_TARGET_DIR:-$root/target}
packages="-p sim-app -p sim-cli -p sim-opcua -p sim-fmi"
if [ "$(uname -s)" = Darwin ]; then
  platform=macos-universal
  fmi=libsim_fmi.dylib
  mkdir -p "$target/release"
  for triple in aarch64-apple-darwin x86_64-apple-darwin; do
    # shellcheck disable=SC2086
    cargo build --release --locked --target "$triple" $packages
  done
  for file in sim-app sim-cli sim-opcua $fmi; do
    lipo -create -output "$target/release/$file"       "$target/aarch64-apple-darwin/release/$file" "$target/x86_64-apple-darwin/release/$file"
  done
else
  platform=linux-x64
  fmi=libsim_fmi.so
  # shellcheck disable=SC2086
  cargo build --release --locked $packages
fi

name="kinematics-simulation-$version-$platform"
stage="$root/dist/$name"
rm -rf "$stage"
mkdir -p "$stage/bin" "$stage/configs/t-gantry" "$stage/configs/single-axis"

for file in sim-app sim-cli sim-opcua $fmi; do
  cp "$target/release/$file" "$stage/bin/"
done
cp README.md CHANGELOG.md LICENSE-MIT LICENSE-APACHE THIRD-PARTY-LICENSES.md "$stage/"
cp -R docs "$stage/docs"
# File names kept: the app picks CAD meshes by config file name. The meshes (vendor CAD) are not packed.
cp crates/machine-t-gantry/default.toml crates/machine-t-gantry/exct-30.toml "$stage/configs/t-gantry/"
cp crates/machine-single-axis/default.toml "$stage/configs/single-axis/"

tar -C "$root/dist" -czf "$root/dist/$name.tar.gz" "$name"
echo "$root/dist/$name.tar.gz"

# The Debian package from the same folder: /opt/kinematics-simulation, the programs linked into /usr/bin, a menu entry.
if [ "$platform" = linux-x64 ] && command -v dpkg-deb >/dev/null; then
  # In a temporary folder: on a Windows drive under WSL every file is mode 777, which dpkg-deb refuses.
  deb=$(mktemp -d)
  chmod 755 "$deb"
  mkdir -p "$deb/DEBIAN" "$deb/opt" "$deb/usr/bin" "$deb/usr/share/applications"
  cp -R "$stage" "$deb/opt/kinematics-simulation"
  find "$deb/opt" -type f ! -path '*/bin/*' -exec chmod 644 {} +
  for program in sim-app sim-cli sim-opcua; do
    ln -s "/opt/kinematics-simulation/bin/$program" "$deb/usr/bin/$program"
  done
  cat > "$deb/usr/share/applications/kinematics-simulation.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Kinematics Simulation
Comment=Physics simulator of belt-driven gantries
Exec=/opt/kinematics-simulation/bin/sim-app
Categories=Science;Engineering;
EOF
  # Debian sorts "~" before the release: 1.0.0-rc.1 becomes 1.0.0~rc.1.
  cat > "$deb/DEBIAN/control" <<EOF
Package: kinematics-simulation
Version: $(echo "$version" | tr '-' '~')
Architecture: amd64
Maintainer: Kinematics Simulation
Depends: libc6, libasound2t64 | libasound2, libudev1, libx11-6, libxkbcommon-x11-0, libvulkan1
Description: Physics simulator of belt-driven gantries
 Desktop app, command-line tool, OPC UA server and FMI 2.0 library.
EOF
  dpkg-deb --root-owner-group --build "$deb" "$root/dist/kinematics-simulation_$(echo "$version" | tr '-' '~')_amd64.deb"
  rm -rf "$deb"
else
  echo "no .deb: built on Linux with dpkg-deb only" >&2
fi
rm -rf "$stage"
