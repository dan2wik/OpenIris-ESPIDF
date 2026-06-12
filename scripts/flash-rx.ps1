# Flash the XIAO ESP32-S3 RX dongle, handling every device state:
#  - app running (CDC on $RuntimePort): trigger the bootloader with a
#    1200-baud touch, wait for the bootloader port to appear
#  - already in ROM download mode on $FlashPort: flash directly
# Ends with an RTC-watchdog reset so the app boots after programming
# (a plain hard reset over USB-Serial/JTAG re-enters download mode on Windows).

$ErrorActionPreference = 'Continue'
$FlashPort   = 'COM24'  # "USB JTAG/serial debug unit" (ROM bootloader / BOOT button)
$RuntimePort = 'COM3'   # "OpenIris Serial" (runtime CDC in the camera composite)

. C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1
Set-Location (Split-Path $PSScriptRoot -Parent)

$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($ports -notcontains $FlashPort) {
    if ($ports -contains $RuntimePort) {
        # Touch baud 12345 reboots the firmware into the bootloader (1200 is
        # avoided: Windows sermouse probes new ports at 1200 and kept
        # triggering it). 1200 is retried as fallback for older firmware.
        $triggered = $false
        $lastReason = ''
        foreach ($baud in 12345, 12345, 1200) {
            $sp = New-Object System.IO.Ports.SerialPort $RuntimePort, $baud
            try {
                $sp.DtrEnable = $true
                $sp.Open()
                Start-Sleep -Milliseconds 200
                $sp.DtrEnable = $false   # DTR drop at touch baud -> bootloader
                Start-Sleep -Milliseconds 100
                $triggered = $true
            } catch {
                $lastReason = $_.Exception.InnerException.Message ?? $_.Exception.Message
                Start-Sleep -Milliseconds 500
            } finally {
                try { $sp.Close() } catch {} # port may vanish as the device resets
            }
            if ($triggered) {
                # wait briefly; if the port is gone the device is rebooting
                Start-Sleep -Milliseconds 500
                if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $RuntimePort) { break }
                $triggered = $false  # device didn't reset (wrong baud?) - try next
            }
        }
        if (-not $triggered) {
            Write-Host "ERROR: Could not trigger the bootloader via ${RuntimePort}: $lastReason" -ForegroundColor Red
            if ($lastReason -match 'denied') {
                Write-Host "Port is held by another program OR the device is in a failed state (replug to recover)." -ForegroundColor Yellow
            } else {
                Write-Host "The device may be wedged or mid-enumeration - replug it (hold BOOT for direct bootloader access)." -ForegroundColor Yellow
            }
            exit 1
        }
        for ($i = 0; $i -lt 50; $i++) {
            Start-Sleep -Milliseconds 200
            if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $FlashPort) { break }
        }
        if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $FlashPort) {
            Write-Host "ERROR: Bootloader port $FlashPort did not appear after reset trigger." -ForegroundColor Red
            exit 1
        }
    } else {
        Write-Host "ERROR: Neither $FlashPort (bootloader) nor $RuntimePort (runtime CDC) found." -ForegroundColor Red
        Write-Host "Hold BOOT while plugging the dongle in, then retry." -ForegroundColor Yellow
        exit 1
    }
}

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

idf.py -B build_rx -D SDKCONFIG="$PWD\sdkconfig.rx" "-DSDKCONFIG_DEFAULTS=sdkconfig.base_defaults;sdkconfig.board.xiao-esp32s3;sdkconfig.rx_defaults" -p $FlashPort flash
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Restarting into app (RTC watchdog reset)..."
python -m esptool --chip esp32s3 -p $FlashPort --before default_reset --after watchdog_reset chip_id | Out-Null
Write-Host "Done. Dongle should re-enumerate as OpenIris Camera + OpenIris Serial ($RuntimePort)."
