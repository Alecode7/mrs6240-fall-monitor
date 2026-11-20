$ErrorActionPreference = "Stop"
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot "MANIFEST.json") -Raw -Encoding UTF8 | ConvertFrom-Json
$root = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\') + '\'
$checked = 0
foreach ($entry in $manifest.files) {
    $path = [System.IO.Path]::GetFullPath((Join-Path $root $entry.path))
    if (-not $path.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)) { throw "Unsafe manifest path: $($entry.path)" }
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing: $($entry.path)" }
    $file = Get-Item -LiteralPath $path
    if ($file.Length -ne $entry.bytes) { throw "Size mismatch: $($entry.path)" }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.sha256) {
        throw "SHA256 mismatch: $($entry.path)"
    }
    $checked++
    if ($checked % 500 -eq 0) { Write-Output "Verified $checked files..." }
}
Write-Output "PASS: $checked files match the handover manifest."
