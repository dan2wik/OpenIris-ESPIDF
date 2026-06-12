# Flash the Project Babble camera (TX). Its USB-Serial/JTAG port is always
# available, so no bootloader trigger is needed. Ends with an RTC-watchdog
# reset so the app reliably boots after programming (a plain hard reset over
# USB-Serial/JTAG can re-enter download mode on Windows).

$ErrorActionPreference = 'Continue'
$FlashPort = 'COM22'  # Babble board "USB JTAG/serial debug unit"

. C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1
Set-Location (Split-Path $PSScriptRoot -Parent)

# Pre-flight: make sure nothing else (e.g. a stale monitor) is holding the port
try {
    $probe = New-Object System.IO.Ports.SerialPort $FlashPort
    $probe.Open()
    $probe.Close()
} catch {
    Write-Host "ERROR: $FlashPort exists but is busy - another program has it open." -ForegroundColor Red
    Get-CimInstance Win32_Process |
        Where-Object { $_.Name -match 'python' -and $_.CommandLine -match 'monitor|esptool' } |
        ForEach-Object { Write-Host ("  suspect: PID {0}  {1}" -f $_.ProcessId, $_.CommandLine) -ForegroundColor Yellow }
    Write-Host "Close those processes (or their terminals) and retry." -ForegroundColor Yellow
    exit 1
}

idf.py -B build_tx -D SDKCONFIG="$PWD\sdkconfig.tx" -p $FlashPort flash
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Restarting into app (RTC watchdog reset)..."
python -m esptool --chip esp32s3 -p $FlashPort --before default_reset --after watchdog_reset chip_id | Out-Null
Write-Host "Done."
