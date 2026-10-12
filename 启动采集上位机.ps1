$projectRoot = $PSScriptRoot
$python = Join-Path $projectRoot "fall_capture_station\.venv\Scripts\python.exe"
$app = Join-Path $projectRoot "fall_capture_station\capture_app.py"
$env:MPLCONFIGDIR = Join-Path $projectRoot ".mplconfig"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Python environment is unavailable: $python"
}

Set-Location $projectRoot
& $python $app
