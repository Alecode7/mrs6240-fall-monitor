param(
    [string]$Python,
    [string]$HifDll,
    [string]$Ch347Dll
)
$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$buildTag = Get-Date -Format "yyyyMMdd-HHmmss"
if (-not $Python) { $Python = Join-Path $projectRoot ".build\portable-env\Scripts\python.exe" }
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Create the build environment first: py -3.12 -m venv .build\portable-env"
}
& $Python -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot "requirements-build.txt")
if ($LASTEXITCODE -ne 0) { throw "Build dependency installation failed." }
if ($HifDll) { $env:MRS6240_HIF_DLL = (Resolve-Path -LiteralPath $HifDll).Path }
if ($Ch347Dll) { $env:MRS6240_CH347_DLL = (Resolve-Path -LiteralPath $Ch347Dll).Path }
$releaseRoot = Join-Path $projectRoot "releases\$buildTag"
$workRoot = Join-Path $projectRoot ".build\pyinstaller\$buildTag"
& $Python -m PyInstaller --noconfirm --distpath $releaseRoot --workpath $workRoot (Join-Path $PSScriptRoot "portable.spec")
if ($LASTEXITCODE -ne 0) { throw "EXE packaging failed. See PyInstaller output." }
$package = Join-Path $releaseRoot "MRS6240-Fall-Monitor"
Copy-Item -LiteralPath (Join-Path $PSScriptRoot "USER_GUIDE.txt") -Destination $package
$drivers = New-Item -ItemType Directory -Path (Join-Path $package "drivers")
foreach ($archive in @("ch347driver for uart spi_i2c.zip", "CH343SER for dual uart.ZIP")) {
    $source = Join-Path $projectRoot "SDK\Software_Kit\03_Tool\USB Serial Driver\$archive"
    if (Test-Path -LiteralPath $source) { Copy-Item -LiteralPath $source -Destination $drivers.FullName }
}
$commit = & git -C $projectRoot rev-parse HEAD
$dependencies = @(& $Python -m pip freeze)
$hashes = @(Get-ChildItem -LiteralPath $package -File -Recurse | ForEach-Object {
    [ordered]@{ path = $_.FullName.Substring($package.Length + 1); sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
})
$manifest = [ordered]@{ built_at = (Get-Date -Format "o"); source_commit = $commit;
    source_has_local_changes = [bool](& git -C $projectRoot status --porcelain);
    platform = "Windows x64"; dependencies = $dependencies; files = $hashes }
$manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $package "build-manifest.json") -Encoding utf8
Write-Output "Portable application folder: $package"
