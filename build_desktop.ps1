param(
    [string]$Python = "$env:USERPROFILE\miniconda3\python.exe",
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (-not (Test-Path $Python)) {
        throw 'Select Python with pythonocc-core, numpy, trimesh, pywebview, and PyInstaller using -Python.'
    }
    if (-not $SkipBuild) {
        & cargo build --release -p scancraft-kernel
        if ($LASTEXITCODE -ne 0) { throw 'Converter build failed.' }
        & npm.cmd run build
        if ($LASTEXITCODE -ne 0) { throw 'Interface build failed.' }
        & $Python -m PyInstaller --noconfirm --distpath dist --workpath target/desktop-build packaging/ScanCraft.spec
        if ($LASTEXITCODE -ne 0) { throw 'Executable packaging failed.' }
    }
    & .\dist\ScanCraft\scancraft-cad.exe --check-runtime
    if ($LASTEXITCODE -ne 0) { throw 'The bundled CAD runtime failed its startup check.' }
    $InstallerCompiler = @(
        "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
        "$env:ProgramFiles\Inno Setup 6\ISCC.exe"
    ) | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $InstallerCompiler) {
        throw 'Install Inno Setup 6 to build the installer. The portable executable is in dist\ScanCraft.'
    }
    & $InstallerCompiler /Qp packaging/ScanCraft.iss
    if ($LASTEXITCODE -ne 0) { throw 'Installer build failed.' }
    Get-Item dist/ScanCraft/ScanCraft.exe, dist/installer/ScanCraft-Setup-0.2.1.exe |
        Select-Object FullName, Length
}
finally {
    Pop-Location
}