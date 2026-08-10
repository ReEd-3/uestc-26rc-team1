# ============================================================================
# fix_cubemx_cmake_project.ps1
#
# Fix broken VSCode configurations in STM32CubeMX-generated CMake projects.
#
# Usage (run from project root after CubeMX code generation):
#   powershell -ExecutionPolicy Bypass -File fix_cubemx_cmake_project.ps1
#
# This script fixes 3 known issues:
#   1. STM32 VSCode extension wrongly adds -DCMAKE_COMMAND=cube-cmake
#      to cmake.configureArgs, which breaks CMake configuration.
#   2. No .vscode/launch.json for ST-Link debugging.
#   3. clangd can't find compile_commands.json in build/ subdirectory.
# ============================================================================

param(
    [string]$ProjectPath = "."
)

Set-Location $ProjectPath
$ErrorActionPreference = "Continue"

Write-Host "=== STM32CubeMX CMake VSCode Config Fix ===" -ForegroundColor Cyan
Write-Host "Project: $(Get-Location)`n"

# ---- 1. Read device info from CubeMX files ----
$deviceName = ""
$deviceCore = ""

# Read device info from .ioc file (primary) or .mxproject (fallback)
$iocFile = Get-ChildItem "*.ioc" | Select-Object -First 1
if ($iocFile) {
    $iocContent = Get-Content $iocFile.FullName -Raw
    # Use CPN (full part number, e.g. STM32H723ZGT6) — debug extension needs this
    if ($iocContent -match "(?<!\w)Mcu\.CPN=(\S+)")      { $deviceName = $Matches[1] }
    elseif ($iocContent -match "(?<!\w)Mcu\.Name=(\S+)")  { $deviceName = $Matches[1] }
    if ($iocContent -match "CORTEX_M7")                 { $deviceCore = "Cortex-M7" }
    elseif ($iocContent -match "CORTEX_M4")             { $deviceCore = "Cortex-M4" }
    elseif ($iocContent -match "CORTEX_M33")            { $deviceCore = "Cortex-M33" }
    elseif ($iocContent -match "CORTEX_M0PLUS")         { $deviceCore = "Cortex-M0+" }
    elseif ($iocContent -match "CORTEX_M85")            { $deviceCore = "Cortex-M85" }
    elseif ($iocContent -match "CORTEX_M3")             { $deviceCore = "Cortex-M3" }
    elseif ($iocContent -match "CORTEX_M0[^P]")         { $deviceCore = "Cortex-M0" }
}

# Fallback: try .mxproject
if ((-not $deviceName) -and (Test-Path ".mxproject")) {
    $mx = Get-Content ".mxproject" -Raw
    if ($mx -match "(?<!\w)Mcu\.Name=(\S+)") { $deviceName = $Matches[1] }
}

# Last resort fallback
if (-not $deviceName) {
    if ($iocFile) { $deviceName = [IO.Path]::GetFileNameWithoutExtension($iocFile.Name) }
}
if (-not $deviceName) { $deviceName = "STM32H723ZGTx" }
if (-not $deviceCore) { $deviceCore = "Cortex-M7" }

$projectName = (Get-Item .).Name
Write-Host "Device: $deviceName | Core: $deviceCore | Project: $projectName`n"

# Common UTF-8 encoder (no BOM) for all file writes
$utf8nobom = New-Object System.Text.UTF8Encoding $false

# ---- 2. Ensure .vscode and .settings directories exist ----
$vscodeDir = ".vscode"
if (-not (Test-Path $vscodeDir)) {
    New-Item -ItemType Directory -Path $vscodeDir -Force | Out-Null
}

# ---- 3. Write correct .vscode/settings.json ----
Write-Host "[1/4] Writing .settings/ide.store.json..." -ForegroundColor Yellow

# CubeMX with CMake toolchain doesn't create .settings — but the STM32 VSCode
# extension needs it to recognize the project as a CubeMX project
$settingsDir = ".settings"
if (-not (Test-Path $settingsDir)) {
    New-Item -ItemType Directory -Path $settingsDir -Force | Out-Null
}
$ideStoreJson = @"
{
  "source": {
    "sourceType": "STM32CubeMX"
  },
  "device": "${deviceName}",
  "core": "${deviceCore}",
  "order": 0,
  "toolchain": "GCC"
}
"@
[System.IO.File]::WriteAllText("$settingsDir\ide.store.json", $ideStoreJson, $utf8nobom)

# Also write bundles.store.json — the STM32 extension needs this to manage toolchains
$bundlesJson = @"
{
  "bundles": [
    { "name": "cmake",               "version": "4.3.1+st.1" },
    { "name": "ninja",               "version": "1.13.2+st.1" },
    { "name": "gnu-tools-for-stm32", "version": "14.3.1+st.2" },
    { "name": "st-arm-clangd",       "version": "21.1.0+st.2" }
  ]
}
"@
[System.IO.File]::WriteAllText("$settingsDir\bundles.store.json", $bundlesJson, $utf8nobom)
Write-Host "  Done: .settings/ide.store.json + bundles.store.json" -ForegroundColor Green

Write-Host "[2/4] Writing .vscode/settings.json..." -ForegroundColor Yellow

$settingsText = @"
{
    "cmake.cmakePath": "cube-cmake",
    "cmake.preferredGenerators": [
        "Ninja"
    ],
    "clangd.arguments": [
        "--compile-commands-dir=build/Debug"
    ]
}
"@
[System.IO.File]::WriteAllText("$vscodeDir\settings.json", $settingsText, $utf8nobom)
Write-Host "  Done: .vscode/settings.json" -ForegroundColor Green

# ---- 4. Write .vscode/launch.json ----
Write-Host "[3/4] Writing .vscode/launch.json..." -ForegroundColor Yellow

$launchText = @"
{
    "version": "0.2.0",
    "configurations": [
        {
            "type": "stlinkgdbtarget",
            "request": "launch",
            "name": "STM32Cube: Launch ST-Link GDB Server",
            "cwd": "`${workspaceFolder}",
            "preBuild": "`${command:st-stm32-ide-debug-launch.build}",
            "runEntry": "main",
            "imagesAndSymbols": [
                {
                    "imageFileName": "`${workspaceFolder}/build/Debug/${projectName}.elf"
                }
            ],
            "deviceName": "${deviceName}",
            "deviceCore": "${deviceCore}"
        }
    ]
}
"@
[System.IO.File]::WriteAllText("$vscodeDir\launch.json", $launchText, $utf8nobom)
Write-Host "  Done: .vscode/launch.json" -ForegroundColor Green

# ---- 5. compile_commands.json ----
Write-Host "[4/4] compile_commands.json..." -ForegroundColor Yellow

$src = "build/Debug/compile_commands.json"
if (Test-Path $src) {
    if (-not (Test-Path "compile_commands.json")) {
        try {
            New-Item -ItemType SymbolicLink -Path "compile_commands.json" -Target $src -ErrorAction Stop | Out-Null
            Write-Host "  Done: symlink created" -ForegroundColor Green
        } catch {
            Copy-Item $src "compile_commands.json"
            Write-Host "  Done: copied (enable Windows Developer Mode for symlinks)" -ForegroundColor DarkYellow
        }
    } else {
        Write-Host "  Already exists, skipping" -ForegroundColor Gray
    }
} else {
    Write-Host "  Not yet configured. Run CMake Configure first." -ForegroundColor DarkYellow
}

# ---- Done ----
Write-Host "`n=== Fix Complete ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor White
Write-Host "  1. Open project in VSCode" -ForegroundColor White
Write-Host "  2. CMake: Configure (Ctrl+Shift+P)" -ForegroundColor White
Write-Host "  3. Press F5 to debug" -ForegroundColor White
