param([Parameter(Mandatory=$true)][string]$Package)
$ErrorActionPreference = "Stop"
$packagePath = (Resolve-Path -LiteralPath $Package).Path
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$verificationRoot = Join-Path $projectRoot (".build\portable-verification\" + (Get-Date -Format "yyyyMMdd-HHmmss"))
New-Item -ItemType Directory -Path $verificationRoot | Out-Null
$copiedPackage = Join-Path $verificationRoot ((-join [char[]](0x4E2D,0x6587,0x76EE,0x5F55)) + " portable copy")
Copy-Item -LiteralPath $packagePath -Destination $copiedPackage -Recurse

function Invoke-Check([string]$Executable, [string]$Switch, [string]$ReportName) {
    $report = Join-Path $verificationRoot $ReportName
    $process = Start-Process -FilePath (Join-Path $copiedPackage $Executable) `
        -ArgumentList @($Switch, ('"{0}"' -f $report)) `
        -WorkingDirectory $copiedPackage -WindowStyle Hidden -PassThru
    if (-not $process.WaitForExit(30000)) {
        Stop-Process -Id $process.Id
        throw "程序在 30 秒内未完成验证：$Executable $Switch"
    }
    if (-not (Test-Path -LiteralPath $report)) { throw "未生成验证报告：$report" }
    $result = Get-Content -LiteralPath $report -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($process.ExitCode -ne 0 -or -not $result.passed) {
        throw "验证失败，请查看报告：$report"
    }
    Write-Output "验证通过：$Executable $Switch，报告：$report"
}

$previousPath = $env:PATH
$previousPythonPath = $env:PYTHONPATH
$previousPythonHome = $env:PYTHONHOME
try {
    # 清除 Python 相关环境路径，验证便携版可以独立于开发环境运行。
    $env:PATH = "$env:WINDIR\System32;$env:WINDIR"
    $env:PYTHONPATH = ""
    $env:PYTHONHOME = ""
    Invoke-Check "MRS6240Monitor.exe" "--self-test" "dependencies.json"
    Invoke-Check "MRS6240Monitor.exe" "--smoke-test" "monitor-ui.json"
    Invoke-Check "MRS6240Engineering.exe" "--smoke-test" "engineering-ui.json"
} finally {
    $env:PATH = $previousPath
    $env:PYTHONPATH = $previousPythonPath
    $env:PYTHONHOME = $previousPythonHome
}
Write-Output "便携版复制验证通过，验证副本：$copiedPackage"
