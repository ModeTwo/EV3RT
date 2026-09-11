param([switch]$CheckOnly)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.IO.Compression.FileSystem
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$manifest = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'manifest.json') | ConvertFrom-Json
function TargetPath([string]$relative) {
    $path = [IO.Path]::GetFullPath((Join-Path $root $relative))
    if (-not $path.StartsWith($root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) { throw 'Path outside repository' }
    return $path
}
# Verify all entries before changing files.
foreach ($item in $manifest.files) {
    $path = TargetPath $item.path
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing current file: $($item.path)" }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $item.after_sha256) { throw "Changed since snapshot: $($item.path)" }
}
$archive = [IO.Compression.ZipFile]::OpenRead((Join-Path $PSScriptRoot 'before.zip'))
try {
# Verify all entries before changing files.
    $originals = @{}
    foreach ($item in $manifest.files) {
        if ($null -eq $item.before_sha256) { continue }
        $entry = $archive.GetEntry($item.path)
        if ($null -eq $entry) { throw "Backup missing: $($item.path)" }
        $stream = $entry.Open()
        $memory = New-Object IO.MemoryStream
        try { $stream.CopyTo($memory); $bytes = $memory.ToArray() } finally { $stream.Dispose(); $memory.Dispose() }
        $sha = [Security.Cryptography.SHA256]::Create()
        try { $hash = [BitConverter]::ToString($sha.ComputeHash($bytes)).Replace('-', '') } finally { $sha.Dispose() }
        if ($hash -ne $item.before_sha256) { throw "Backup hash mismatch: $($item.path)" }
        $originals[$item.path] = $bytes
    }
} finally { $archive.Dispose() }
if ($CheckOnly) { Write-Output 'Restore check passed; no files changed.'; exit 0 }
# Verify all entries before changing files.
$saved = Join-Path $PSScriptRoot ('replaced-' + [Guid]::NewGuid().ToString('N') + '.zip')
$archive = [IO.Compression.ZipFile]::Open($saved, [IO.Compression.ZipArchiveMode]::Create)
try {
    foreach ($item in $manifest.files) {
        [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive, (TargetPath $item.path), $item.path) | Out-Null
    }
} finally { $archive.Dispose() }
foreach ($item in $manifest.files) {
    $path = TargetPath $item.path
    if ($null -eq $item.before_sha256) {
        Remove-Item -LiteralPath $path
    } else {
        [IO.File]::WriteAllBytes($path, $originals[$item.path])
    }
}
Write-Output "Restored pre-migration files. Replaced version saved in $saved"
