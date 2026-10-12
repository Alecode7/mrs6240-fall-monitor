$projectRoot = $PSScriptRoot
$python = Join-Path $projectRoot "fall_capture_station\.venv\Scripts\python.exe"
$app = Join-Path $projectRoot "fall_detection\serial_event_live_visualizer.py"
$env:MPLCONFIGDIR = Join-Path $projectRoot ".mplconfig"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Python environment is unavailable: $python"
}

Set-Location $projectRoot
& $python $app --source device --spi-index 0 --spi-freq-mode 0 --serial-port COM3 --serial-baud 115200
