#Requires -Version 5.1
<#
.SYNOPSIS
  Live Windows Bluetooth pairing/SPP diagnostic: Event Viewer + Plug and Play.
.DESCRIPTION
  This collects WINDOWS-SIDE events while pairing/opening the JDY-33 SPP
  outgoing COM. It does NOT open COM ports, send AT commands or alter pairing.
  A Windows PnP Status=OK does not prove an active Bluetooth radio link.
  Event channel availability varies by Windows build; quiet logs do not prove
  that pairing or RFCOMM negotiation succeeded.
.EXAMPLE
  powershell -NoProfile -ExecutionPolicy Bypass -File tools\windows_bluetooth_link_debug.ps1 -DurationSeconds 90
#>
[CmdletBinding()]
param(
    [ValidateRange(5, 3600)][int]$DurationSeconds = 90,
    [ValidateRange(200, 10000)][int]$IntervalMs = 900,
    [string]$DevicePattern = 'Nezumi|JDY-33|Serial over Bluetooth',
    [string]$OutputDir = '',
    [switch]$SelfTest
)

if ($SelfTest) {
    $parsed = [regex]::IsMatch('Nezumi_Chan_SPP', 'Nezumi|JDY-33')
    if (-not $parsed) { throw 'Self-test failed' }
    Write-Output 'SELFTEST PASS: Windows Bluetooth debug monitor command parsed.'
    return
}

$ErrorActionPreference = 'Continue'
if (-not $OutputDir) {
    $OutputDir = Join-Path (Split-Path -Parent $PSScriptRoot) 'results\classic_spp\windows'
}
$OutputDir = [IO.Path]::GetFullPath($OutputDir)
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$log = Join-Path $OutputDir ("windows_bt_" + $stamp + '.jsonl')
$script:outputFile = $log
$script:started = Get-Date

function Emit([string]$Kind, [string]$Note, [object]$Data = $null) {
    $now = Get-Date
    $payload = [ordered]@{
        time = $now.ToString('o')
        elapsed_s = [math]::Round(($now - $script:started).TotalSeconds, 3)
        kind = $Kind
        detail = $Note
    }
    if ($null -ne $Data) { $payload['data'] = $Data }
    $json = ConvertTo-Json -InputObject $payload -Compress -Depth 6
    Add-Content -Path $script:outputFile -Value $json -Encoding UTF8
    Write-Host ('[{0:HH:mm:ss.fff}] [{1}] {2}' -f $now, $Kind, $Note)
}

Emit 'START' ("Windows Bluetooth PnP/EventLog monitor for {0}s" -f $DurationSeconds) @{
    windows = [Environment]::OSVersion.VersionString
    filter = $DevicePattern
    file = $log
}
Emit 'NOTE' 'No COM port opened by this script; Windows may show paired-but-Not-connected until an app opens outgoing SPP COM.'
Emit 'NOTE' 'PnP Status=OK only indicates device/driver status, not an active RFCOMM session.'

# Enumerate only the channels actually installed and enabled on this PC.
$channelList = New-Object System.Collections.Generic.List[string]
foreach ($wildcard in @('*Bluetooth*', '*BTH*')) {
    try {
        $matches = @(Get-WinEvent -ListLog $wildcard -ErrorAction Stop)
        foreach ($entry in $matches) {
            if ($entry.IsEnabled -and -not $channelList.Contains($entry.LogName)) {
                [void]$channelList.Add($entry.LogName)
            }
        }
    } catch {
        # A Windows build can have no matching manifest-backed channels.
    }
}
foreach ($name in @('Microsoft-Windows-Kernel-PnP/Configuration', 'System')) {
    try {
        $entry = Get-WinEvent -ListLog $name -ErrorAction Stop
        if ($entry.IsEnabled -and -not $channelList.Contains($name)) {
            [void]$channelList.Add($name)
        }
    } catch { }
}
Emit 'CHANNELS' ("Monitoring installed enabled channels: " + ($channelList -join ', ')) @{
    channels = @($channelList)
}
if ($channelList.Count -eq 0) {
    Emit 'WARNING' 'No readable event channels. Run PowerShell as Administrator for more visibility.'
}

# Snapshot hardware and COM enumerators; changes can expose a pairing failure
# or a new RFCOMM COM mapping, but never prove the live radio link state.
function Read-PnpSnapshot {
    $out = New-Object System.Collections.Generic.List[object]
    try {
        $devices = @(Get-CimInstance -ClassName Win32_PnPEntity -ErrorAction Stop)
        foreach ($device in $devices) {
            $name = [string]$device.Name
            $id = [string]$device.PNPDeviceID
            $cls = [string]$device.PNPClass
            if ($name -match $DevicePattern -or
                (($cls -eq 'Ports' -or $cls -eq 'Bluetooth') -and
                 ($name -match 'Bluetooth' -or $id -match 'BTHENUM'))) {
                $out.Add([ordered]@{
                    name = $name
                    id = $id
                    class = $cls
                    status = [string]$device.Status
                })
            }
        }
    } catch {
        Emit 'WARNING' ('Could not query PnP inventory: ' + $_.Exception.Message)
    }
    return @($out | Sort-Object name, id)
}

$lastPnp = ''
$seen = New-Object 'System.Collections.Generic.HashSet[string]'
$nextPnp = [datetime]::MinValue
$end = $script:started.AddSeconds($DurationSeconds)
try {
    while ((Get-Date) -lt $end) {
        $now = Get-Date
        if ($now -ge $nextPnp) {
            $snapshot = @(Read-PnpSnapshot)
            $serialized = ConvertTo-Json -InputObject $snapshot -Compress -Depth 5
            if ($serialized -ne $lastPnp) {
                $lastPnp = $serialized
                Emit 'PNP_CHANGE' ('Bluetooth device / COM inventory changed: {0} entries' -f $snapshot.Count) $snapshot
                foreach ($item in $snapshot) {
                    Emit 'DEVICE' ("{0} | {1} | {2}" -f $item.status, $item.class, $item.name) @{
                        id = $item.id
                    }
                }
            }
            $nextPnp = $now.AddSeconds(3)
        }

        foreach ($channel in $channelList) {
            try {
                # Keep a small overlapping window and de-duplicate by record ID.
                # Covers the previous loop even if polling runs a little late.
                $since = (Get-Date).AddSeconds(-6)
                $events = @(Get-WinEvent -FilterHashtable @{
                    LogName = $channel
                    StartTime = $since
                } -MaxEvents 120 -ErrorAction Stop)
                foreach ($e in ($events | Sort-Object TimeCreated)) {
                    if ($e.TimeCreated -lt $script:started) { continue }
                    $key = [string]$channel + ':' + [string]$e.RecordId
                    if (-not $seen.Add($key)) { continue }
                    if ($channel -eq 'System' -and
                        [string]$e.ProviderName -notmatch '(?i)BTH|Bluetooth|Kernel-PnP|UserPnp|DeviceSetup') {
                        continue
                    }
                    $message = ([string]$e.Message -replace '[\r\n]+', ' ').Trim()
                    if ($message.Length -gt 350) { $message = $message.Substring(0, 350) + ' ...' }
                    Emit 'WIN_EVENT' ("{0} | {1} | ID={2} | {3}" -f
                        $channel, $e.ProviderName, $e.Id, $message) @{
                        channel = $channel
                        provider = $e.ProviderName
                        event_id = $e.Id
                        level = $e.LevelDisplayName
                        record_id = $e.RecordId
                        message = [string]$e.Message
                    }
                }
            } catch {
                # Empty channel/no matching events are normal on many PCs.
            }
        }
        Start-Sleep -Milliseconds $IntervalMs
    }
} finally {
    Emit 'FINISH' ('Live monitor ended. Log: ' + $log) @{
        note = 'Some failures require Microsoft Bluetooth stack WPR/ETW tracing.'
    }
}
Write-Host ('Log saved: ' + $log)
