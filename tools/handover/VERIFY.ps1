$ErrorActionPreference = "Stop"
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot "MANIFEST.json") -Raw -Encoding UTF8 | ConvertFrom-Json
$root = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\') + '\'
$checked = 0
foreach ($entry in $manifest.files) {
    $path = [System.IO.Path]::GetFullPath((Join-Path $root $entry.path))
    if (-not $path.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)) { throw "清单路径超出交接目录：$($entry.path)" }
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "缺少文件：$($entry.path)" }
    $file = Get-Item -LiteralPath $path
    if ($file.Length -ne $entry.bytes) { throw "文件大小不一致：$($entry.path)" }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.sha256) {
        throw "文件 SHA256 不一致：$($entry.path)"
    }
    $checked++
    if ($checked % 500 -eq 0) { Write-Output "已校验 $checked 个文件……" }
}
Write-Output "校验通过：$checked 个文件与交接清单一致。"
