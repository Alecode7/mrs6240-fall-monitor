param(
    [ValidateSet("engineering", "monitor")]
    [string]$UiMode = "engineering"
)
$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$exeName = if ($UiMode -eq "monitor") { "MRS6240Monitor.exe" } else { "MRS6240Engineering.exe" }
$releaseRoot = Join-Path $projectRoot "releases"
if (Test-Path -LiteralPath $releaseRoot) {
    foreach ($release in (Get-ChildItem -LiteralPath $releaseRoot -Directory | Sort-Object Name -Descending)) {
        $package = Join-Path $release.FullName "MRS6240-Fall-Monitor"
        $exe = Join-Path $package $exeName
        if ((Test-Path -LiteralPath $exe) -and
            (Test-Path -LiteralPath (Join-Path $package "build-manifest.json")) -and
            (Test-Path -LiteralPath (Join-Path $package "_internal\python312.dll"))) {
            Start-Process -FilePath $exe -WorkingDirectory $package | Out-Null
            return
        }
    }
}

# 基础 Python 被卸载后，虚拟环境中的启动文件可能仍然存在，必须实际检查能否运行。
foreach ($relativePython in @("fall_detection\.venv\Scripts\python.exe", ".build\portable-env\Scripts\python.exe")) {
    $python = Join-Path $projectRoot $relativePython
    if (-not (Test-Path -LiteralPath $python)) { continue }
    try {
        & $python -c "import tkinter, matplotlib, serial, clr" 2>$null | Out-Null
        if ($LASTEXITCODE -ne 0) { continue }
    } catch { continue }
    $env:MPLCONFIGDIR = Join-Path $projectRoot ".mplconfig"
    Push-Location $projectRoot
    try {
        & $python (Join-Path $projectRoot "fall_detection\serial_event_live_visualizer.py") `
            --source device --ui-mode $UiMode --spi-index 0 --spi-freq-mode 0 --serial-port COM3 --serial-baud 115200
    } finally { Pop-Location }
    return
}
throw "未找到完整便携版或可用的 Python 环境。请先构建便携版，或重建 fall_detection\.venv 并安装依赖。"
