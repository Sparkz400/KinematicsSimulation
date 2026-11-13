# Builds the release binaries and packs them, unsigned, into dist/kinematics-simulation-<version>-windows-x64.zip
# with README, licences, docs, the three example configs and THIRD-PARTY-LICENSES.md.
# Run from anywhere: pwsh tools/package.ps1. Honours CARGO_TARGET_DIR.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$version = (Select-String -Path Cargo.toml -Pattern '^version = "(.+)"').Matches[0].Groups[1].Value
cargo build --release --locked -p sim-app -p sim-cli -p sim-opcua -p sim-fmi
if ($LASTEXITCODE -ne 0) { throw "cargo build failed" }

$target = if ($env:CARGO_TARGET_DIR) { $env:CARGO_TARGET_DIR } else { Join-Path $root 'target' }
$name = "kinematics-simulation-$version-windows-x64"
$stage = Join-Path $root "dist/$name"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force "$stage/bin", "$stage/configs/t-gantry", "$stage/configs/single-axis" | Out-Null

foreach ($file in 'sim-app.exe', 'sim-cli.exe', 'sim-opcua.exe', 'sim_fmi.dll') {
    Copy-Item (Join-Path $target "release/$file") "$stage/bin/"
}
Copy-Item README.md, CHANGELOG.md, LICENSE-MIT, LICENSE-APACHE, THIRD-PARTY-LICENSES.md $stage
Copy-Item -Recurse docs "$stage/docs"
# File names kept: the app picks CAD meshes by config file name. The meshes themselves (vendor CAD, not in the
# repository) are not packed.
Copy-Item crates/machine-t-gantry/default.toml, crates/machine-t-gantry/exct-30.toml "$stage/configs/t-gantry/"
Copy-Item crates/machine-single-axis/default.toml "$stage/configs/single-axis/"

$zip = Join-Path $root "dist/$name.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path $stage -DestinationPath $zip
Write-Output $zip

# The MSI from the same folder (tools/installer.wxs), when WiX 5 is installed: dotnet tool install --global wix --version 5.0.2
if (Get-Command wix -ErrorAction SilentlyContinue) {
    $msi = Join-Path $root "dist/$name.msi"
    # An MSI version is numbers only: 1.0.0-rc.1 installs as 1.0.0.
    $msiVersion = ($version -split '-')[0]
    wix build tools/installer.wxs -arch x64 -d "Version=$msiVersion" -d "Stage=$stage" -o $msi
    if ($LASTEXITCODE -ne 0) { throw "wix build failed" }
    Remove-Item (Join-Path $root "dist/$name.wixpdb") -ErrorAction SilentlyContinue
    Write-Output $msi
} else {
    Write-Warning "wix not found: no MSI (dotnet tool install --global wix --version 5.0.2)"
}
Remove-Item -Recurse -Force $stage
