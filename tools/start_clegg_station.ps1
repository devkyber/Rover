#Requires -Version 5.1
param(
    [ValidateRange(1024, 65535)] [int] $Port = 8766,
    [string] $CredentialPath,
    [string] $ContextHost,
    [switch] $NoBrowser,
    [switch] $NoPlot,
    [switch] $Restart,
    [switch] $DryRun,
    [switch] $CheckOwner
)
# Run with Windows PowerShell 5.1. Reception and recording only; no Rover commands.
$ErrorActionPreference = 'Stop'
$cleggRoot = [IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$cleggPython = Join-Path $cleggRoot '.venv\Scripts\python.exe'
$cleggStation = Join-Path $cleggRoot 'station\station.py'
$cleggHotspot = Join-Path $PSScriptRoot 'clegg_hotspot.ps1'
$cleggVenvConfig = Join-Path $cleggRoot '.venv\pyvenv.cfg'
if (-not $CredentialPath) {
    $CredentialPath = Join-Path $cleggRoot 'clegg_wifi.local.md'
}
foreach ($cleggRequired in @($cleggPython, $cleggStation, $cleggHotspot, $cleggVenvConfig, $CredentialPath)) {
    if (-not (Test-Path -LiteralPath $cleggRequired -PathType Leaf)) {
        throw "Required file is missing: $cleggRequired"
    }
}
$CredentialPath = [IO.Path]::GetFullPath($CredentialPath)
if (-not $CredentialPath.EndsWith('.local.md', [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Use an ignored *.local.md credential file.'
}
if ($ContextHost -and $ContextHost -notmatch '^[a-zA-Z0-9.:-]+$') {
    throw 'ContextHost must be a host name or IP address.'
}

# Read the firmware's static-address setting from its single runtime owner.
# Isolated Python and -B keep this read independent of PYTHONPATH and prevent
# bytecode writes, including during DryRun. No credentials are read here.
$cleggConfigJson = & $cleggPython -I -B -c "import ctypes,json,sys; image=ctypes.create_unicode_buffer(32768); assert ctypes.windll.kernel32.GetModuleFileNameW(None,image,len(image)), 'Cannot resolve Python process image'; sys.path.insert(0,sys.argv[1]); import config; value=getattr(config,'CLEGG_WIFI_STATIC_IP',''); assert value is None or isinstance(value,str), 'CLEGG_WIFI_STATIC_IP must be a string'; print(json.dumps({'static_ip':value or '', 'venv_executable':sys.executable, 'base_executable':sys._base_executable, 'base_prefix':sys.base_prefix, 'process_executable':image.value}))" (Join-Path $cleggRoot 'rover')
if ($LASTEXITCODE -ne 0) { throw 'Cannot read the Clegg static IP from rover/config.py; no network or process was changed.' }
$cleggConfig = ($cleggConfigJson | Out-String).Trim() | ConvertFrom-Json
# Windows venv launchers can redirect to a base-Python child (including the
# Store package's versioned python.exe). Trust that actual image only after
# the invoked venv identity and pyvenv.cfg base executable agree exactly.
$cleggVenvBaseMatch = [regex]::Match([IO.File]::ReadAllText($cleggVenvConfig), '(?m)^executable\s*=\s*(.+?)\s*$')
if (-not $cleggVenvBaseMatch.Success) { throw 'Cannot identify the venv base executable from pyvenv.cfg.' }
$cleggConfiguredBasePython = [IO.Path]::GetFullPath($cleggVenvBaseMatch.Groups[1].Value.Trim())
$cleggResolvedPythonImage = [IO.Path]::GetFullPath([string] $cleggConfig.process_executable)
$cleggResolvedBasePrefix = [IO.Path]::GetFullPath([string] $cleggConfig.base_prefix).TrimEnd('\')
if (-not [string]::Equals([IO.Path]::GetFullPath([string] $cleggConfig.venv_executable), $cleggPython, [StringComparison]::OrdinalIgnoreCase) -or
    -not [string]::Equals([IO.Path]::GetFullPath([string] $cleggConfig.base_executable), $cleggConfiguredBasePython, [StringComparison]::OrdinalIgnoreCase) -or
    -not (Test-Path -LiteralPath $cleggResolvedPythonImage -PathType Leaf) -or
    (-not [string]::Equals($cleggResolvedPythonImage, $cleggPython, [StringComparison]::OrdinalIgnoreCase) -and
        -not [string]::Equals([IO.Path]::GetDirectoryName($cleggResolvedPythonImage).TrimEnd('\'), $cleggResolvedBasePrefix, [StringComparison]::OrdinalIgnoreCase))) {
    throw 'The venv, pyvenv.cfg and actual base Python image do not agree; no network or process was changed.'
}
$cleggTrustedPythonImages = @($cleggPython, $cleggConfiguredBasePython, $cleggResolvedPythonImage)
$cleggConfiguredSensorIP = ([string] $cleggConfig.static_ip).Trim()
if ($cleggConfiguredSensorIP) {
    $cleggParsedSensorIP = $null
    if ($cleggConfiguredSensorIP -notmatch '^\d{1,3}(\.\d{1,3}){3}$' -or
        -not [Net.IPAddress]::TryParse($cleggConfiguredSensorIP, [ref] $cleggParsedSensorIP) -or
        $cleggParsedSensorIP.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
        [Net.IPAddress]::IsLoopback($cleggParsedSensorIP) -or
        $cleggParsedSensorIP.GetAddressBytes()[0] -eq 0 -or
        $cleggParsedSensorIP.GetAddressBytes()[0] -ge 224) {
        throw 'CLEGG_WIFI_STATIC_IP must be an IPv4 host address or empty; no network or process was changed.'
    }
    $cleggConfiguredSensorIP = $cleggParsedSensorIP.ToString()
}
if ($DryRun) {
    # Configuration read only: no WinRT/network queries, process changes, file
    # writes, or credentials read. Hotspot matching remains required in a real run.
    [pscustomobject]@{
        Mode = 'DryRun'; Python = $cleggPython; Station = $cleggStation; Port = $Port
        BasePythonImage = $cleggResolvedPythonImage
        Hotspot = 'Start or reuse matching 2.4 GHz WPA2 session only'
        Receiver = $(if ($cleggConfiguredSensorIP) { 'Configured static IPv4 after hotspot match' } else { 'Unique tethered client IPv4; otherwise UDP discovery' })
        StaticSensorHost = $cleggConfiguredSensorIP
        ContextHost = $ContextHost
        Plot = $(if ($NoPlot) { 'Disabled' } else { 'Separate native Matplotlib window for a new capture' })
        Replace = 'Only this repository station process after exact argument verification'
        LogRoot = (Join-Path $cleggRoot 'logs\clegg_station')
    }
    exit 0
}
if ($PSVersionTable.PSEdition -ne 'Desktop') {
    throw 'Use Windows PowerShell 5.1: powershell.exe -NoProfile -File tools/start_clegg_station.ps1'
}

# Parse actual Windows argv rather than matching a substring in another command.
if (-not ('CleggStationArgv' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class CleggStationArgv {
    [DllImport("shell32.dll", SetLastError=true)]
    static extern IntPtr CommandLineToArgvW([MarshalAs(UnmanagedType.LPWStr)] string command, out int count);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr pointer);
    public static string[] Parse(string command) {
        int count; IntPtr memory = CommandLineToArgvW(command, out count);
        if (memory == IntPtr.Zero) throw new InvalidOperationException("Cannot parse station command line.");
        try {
            string[] result = new string[count];
            for (int i=0; i<count; i++) result[i] = Marshal.PtrToStringUni(Marshal.ReadIntPtr(memory, i*IntPtr.Size));
            return result;
        } finally { LocalFree(memory); }
    }
}
'@
}

function Get-CleggStationOwner {
    $cleggListeners = @(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue)
    if ($cleggListeners.Count -eq 0) { return $null }
    $cleggOwners = @($cleggListeners | Select-Object -ExpandProperty OwningProcess -Unique)
    if ($cleggOwners.Count -ne 1) { throw "Port $Port has multiple owners; no process was changed." }
    $cleggProcess = Get-CimInstance Win32_Process -Filter "ProcessId=$($cleggOwners[0])"
    if (-not $cleggProcess -or -not $cleggProcess.CommandLine -or -not $cleggProcess.ExecutablePath -or
        -not ($cleggTrustedPythonImages | Where-Object { [string]::Equals($_, $cleggProcess.ExecutablePath, [StringComparison]::OrdinalIgnoreCase) })) {
        throw "Port $Port belongs to another or unverifiable process; no process was changed."
    }
    $cleggArgv = [CleggStationArgv]::Parse($cleggProcess.CommandLine)
    if ($cleggArgv.Length -lt 2 -or -not [IO.Path]::IsPathRooted($cleggArgv[0]) -or
        -not [string]::Equals([IO.Path]::GetFullPath($cleggArgv[0]), $cleggPython, [StringComparison]::OrdinalIgnoreCase) -or
        -not [IO.Path]::IsPathRooted($cleggArgv[1]) -or
        -not [string]::Equals([IO.Path]::GetFullPath($cleggArgv[1]), $cleggStation, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Port $Port is not this repository's absolute station command; no process was changed."
    }
    $cleggOptions = @{}
    for ($cleggI = 2; $cleggI -lt $cleggArgv.Length; $cleggI++) {
        $cleggOption = $cleggArgv[$cleggI]
        if ($cleggOptions.ContainsKey($cleggOption)) { throw 'Duplicate station arguments; preserve the existing process.' }
        if ($cleggOption -in @('--clegg', '--no-browser')) { $cleggOptions[$cleggOption] = $true }
        elseif ($cleggOption -in @('--port', '--bind', '--clegg-host', '--rover', '--clegg-context-host', '--clegg-context-port')) {
            $cleggI++
            if ($cleggI -ge $cleggArgv.Length) { throw 'Incomplete station arguments; preserve the existing process.' }
            $cleggOptions[$cleggOption] = $cleggArgv[$cleggI]
        }
        else { throw 'Unrecognized station arguments; preserve the existing process.' }
    }
    $cleggExistingPort = if ($cleggOptions.ContainsKey('--port')) { [int] $cleggOptions['--port'] } else { 8765 }
    if ($cleggExistingPort -ne $Port -or
        ($cleggOptions.ContainsKey('--bind') -and $cleggOptions['--bind'] -ne '127.0.0.1')) {
        throw 'The existing station bind or port differs; preserve the existing process.'
    }
    [pscustomobject]@{ Process = $cleggProcess; Options = $cleggOptions }
}

if ($CheckOwner) {
    # Local listener/process inspection only. Do not query/change the hotspot,
    # read credentials, contact the station API, or stop/start any process.
    $cleggCheckedOwner = Get-CleggStationOwner
    [pscustomobject]@{
        Mode = 'CheckOwner'; Port = $Port; Verified = [bool] $cleggCheckedOwner
        ProcessId = $(if ($cleggCheckedOwner) { $cleggCheckedOwner.Process.ProcessId } else { $null })
        ParentProcessId = $(if ($cleggCheckedOwner) { $cleggCheckedOwner.Process.ParentProcessId } else { $null })
        ExecutablePath = $(if ($cleggCheckedOwner) { $cleggCheckedOwner.Process.ExecutablePath } else { $null })
        Station = $cleggStation
    }
    exit 0
}

# Verify the port before any hotspot change. The guard runs again before replacement.
$cleggExisting = Get-CleggStationOwner
. $cleggHotspot -FunctionsOnly
$cleggCredentialText = [IO.File]::ReadAllText($CredentialPath)
$cleggSsid = [regex]::Match($cleggCredentialText, '(?m)^SSID: (.+)$').Groups[1].Value.Trim()
$cleggSecret = [regex]::Match($cleggCredentialText, '(?m)^Password: (.+)$').Groups[1].Value.Trim()
$cleggCredentialText = $null
if ($cleggSsid.Length -lt 1 -or $cleggSsid.Length -gt 32 -or $cleggSsid -match '[^\x20-\x7e]' -or
    $cleggSecret.Length -lt 8 -or $cleggSecret.Length -gt 63 -or $cleggSecret -match '[^\x20-\x7e]') {
    throw 'Local Wi-Fi credentials are missing or invalid; credentials were not printed.'
}
$cleggManager = Get-CleggTetheringManager
$cleggState = $cleggManager.TetheringOperationalState.ToString()
if ($cleggState -eq 'Off') {
    $cleggStarted = Start-CleggHotspotSession -Ssid $cleggSsid -Passphrase $cleggSecret
    if ($cleggStarted.Status -ne 'Success' -or $cleggStarted.OperationalState -ne 'On') {
        throw "Hotspot start failed: $($cleggStarted.Status). No station process was changed."
    }
}
elseif ($cleggState -eq 'On') {
    $cleggActive = $cleggManager.GetCurrentAccessPointConfiguration()
    $cleggMatches = [string]::Equals($cleggActive.Ssid, $cleggSsid, [StringComparison]::Ordinal) -and
        [string]::Equals($cleggActive.Passphrase, $cleggSecret, [StringComparison]::Ordinal) -and
        $cleggActive.Band.ToString() -eq 'TwoPointFourGigahertz' -and
        $cleggActive.AuthenticationKind.ToString() -eq 'Wpa2'
    $cleggActive = $null
    if (-not $cleggMatches) {
        throw 'Active hotspot differs from the local Clegg session settings; preserved without changes.'
    }
}
else { throw 'Hotspot is transitioning or unavailable; preserved without changes.' }
$cleggSecret = $null

# The managed AP was started or verified above. Prefer the same static address
# embedded in the Nano image even if Windows has not reported a DHCP client IP.
$cleggSensorIP = $cleggConfiguredSensorIP
if (-not $cleggSensorIP) {
    $cleggClientDeadline = [DateTime]::UtcNow.AddSeconds(10)
    do {
        $cleggClientIPs = @(Get-CleggHotspotClients | ForEach-Object { $_.IpAddresses } | Where-Object {
            $cleggAddress = $null
            [Net.IPAddress]::TryParse($_, [ref] $cleggAddress) -and
                $cleggAddress.AddressFamily -eq [Net.Sockets.AddressFamily]::InterNetwork -and
                -not [Net.IPAddress]::IsLoopback($cleggAddress) -and $_ -ne '0.0.0.0'
        } | Select-Object -Unique)
        if ($cleggClientIPs.Count -eq 1) { $cleggSensorIP = [string] $cleggClientIPs[0]; break }
        if ($cleggClientIPs.Count -gt 1) { break } # Never select an arbitrary client.
        Start-Sleep -Milliseconds 250
    } while ([DateTime]::UtcNow -lt $cleggClientDeadline)
}

$cleggUrl = "http://localhost:$Port/"
$cleggReceiverArgs = if ($cleggSensorIP) { @('--clegg-host', $cleggSensorIP) } else { @('--clegg') }
$cleggReuse = $false
$cleggExistingStatus = $null
if ($cleggExisting) {
    $cleggOptions = $cleggExisting.Options
    $cleggSameReceiver = if ($cleggSensorIP) {
        $cleggOptions['--clegg-host'] -eq $cleggSensorIP -and -not $cleggOptions.ContainsKey('--clegg')
    } else { $cleggOptions.ContainsKey('--clegg') }
    $cleggSameContext = -not $ContextHost -or $cleggOptions['--clegg-context-host'] -eq $ContextHost
    try { $cleggExistingStatus = Invoke-RestMethod "http://127.0.0.1:$Port/clegg/status" -TimeoutSec 2 }
    catch { $cleggExistingStatus = $null }
    if (-not $cleggSameContext -and $cleggExistingStatus.connected -eq $true) {
        throw 'The requested context host differs while Nano recording is live. Stop that acquisition before changing the context source.'
    }
    if (-not $Restart -and $cleggExistingStatus -and $cleggExistingStatus.available -eq $true) {
        # WinRT can briefly omit client IPs. Preserve a healthy acquisition even
        # when automatic host selection cannot currently rediscover its address.
        $cleggReuse = $cleggSameContext -and ($cleggSameReceiver -or $cleggExistingStatus.connected -eq $true)
    }
    if (-not $cleggReuse -and $cleggExistingStatus -and $cleggExistingStatus.connected -eq $true) {
        throw 'Existing station is recording live Nano data; preserved. Stop that acquisition before replacing the server.'
    }
}
$cleggLaunchArguments = @(('"' + $cleggStation + '"'), '--port', [string] $Port) + $cleggReceiverArgs
if ($NoBrowser) { $cleggLaunchArguments += '--no-browser' }
$cleggContextAddress = $ContextHost
if (-not $cleggContextAddress -and $cleggExisting -and $cleggExisting.Options.ContainsKey('--clegg-context-host')) {
    $cleggContextAddress = [string] $cleggExisting.Options['--clegg-context-host']
}
if ($cleggContextAddress) {
    if ($cleggContextAddress -notmatch '^[a-zA-Z0-9.:-]+$') { throw 'Cannot safely preserve the existing context address.' }
    $cleggLaunchArguments += @('--clegg-context-host', $cleggContextAddress)
    if ($cleggExisting -and $cleggExisting.Options.ContainsKey('--clegg-context-port')) {
        $cleggContextPort = [int] $cleggExisting.Options['--clegg-context-port']
        if ($cleggContextPort -lt 1 -or $cleggContextPort -gt 65535) { throw 'Invalid existing context port.' }
        $cleggLaunchArguments += @('--clegg-context-port', [string] $cleggContextPort)
    }
}
if ($cleggExisting -and $cleggExisting.Options.ContainsKey('--rover')) {
    $cleggRoverAddress = [string] $cleggExisting.Options['--rover']
    if ($cleggRoverAddress -notmatch '^[a-zA-Z0-9.:-]+$') { throw 'Cannot safely preserve the existing Rover address.' }
    $cleggLaunchArguments += @('--rover', $cleggRoverAddress)
}
if (-not $cleggReuse) {
    $cleggRechecked = Get-CleggStationOwner
    if ($cleggExisting) {
        if (-not $cleggRechecked -or
            $cleggExisting.Process.ProcessId -ne $cleggRechecked.Process.ProcessId -or
            $cleggExisting.Process.CreationDate -ne $cleggRechecked.Process.CreationDate -or
            $cleggExisting.Process.CommandLine -ne $cleggRechecked.Process.CommandLine) {
            throw 'Station ownership changed during setup; no process was stopped.'
        }
        Stop-Process -Id $cleggRechecked.Process.ProcessId -ErrorAction Stop
        $cleggPortDeadline = [DateTime]::UtcNow.AddSeconds(3)
        while (@(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue).Count -and
               [DateTime]::UtcNow -lt $cleggPortDeadline) { Start-Sleep -Milliseconds 100 }
    }
    elseif ($cleggRechecked) { throw 'Another station appeared during setup; preserved without changes.' }
    if (@(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue).Count) {
        throw "Port $Port is still occupied; no replacement was started."
    }
    $cleggLogPath = Join-Path $cleggRoot ('logs\clegg_station\' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmss_fffffffZ') + '_' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
    $null = New-Item -ItemType Directory -Path $cleggLogPath
    $cleggProcess = Start-Process -FilePath $cleggPython -ArgumentList $cleggLaunchArguments -WorkingDirectory $cleggRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $cleggLogPath 'stdout.log') -RedirectStandardError (Join-Path $cleggLogPath 'stderr.log')
    [pscustomobject]@{
        CreatedUtc = [DateTime]::UtcNow.ToString('o'); ProcessId = $cleggProcess.Id; Port = $Port
        ReceiverHost = $cleggSensorIP; Discovery = (-not [bool] $cleggSensorIP)
        ReceiverHostSource = $(if ($cleggConfiguredSensorIP) { 'rover/config.py' } elseif ($cleggSensorIP) { 'hotspot_client' } else { 'udp_discovery' })
        Station = $cleggStation; Python = $cleggPython; Hotspot = 'matching local session'
        ContextHost = $cleggContextAddress
    } | ConvertTo-Json | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $cleggLogPath 'launch.json')
    $cleggReady = $false
    $cleggReadyDeadline = [DateTime]::UtcNow.AddSeconds(5)
    do {
        $cleggProcess.Refresh()
        if ($cleggProcess.HasExited) { throw "Station exited; inspect logs: $cleggLogPath" }
        try { $cleggReady = (Invoke-RestMethod "http://127.0.0.1:$Port/clegg/status" -TimeoutSec 1).available -eq $true }
        catch { $cleggReady = $false }
        if (-not $cleggReady) { Start-Sleep -Milliseconds 200 }
    } while (-not $cleggReady -and [DateTime]::UtcNow -lt $cleggReadyDeadline)
    if (-not $cleggReady) { throw "Station API did not become ready; inspect logs: $cleggLogPath" }
}
Write-Output "Station: $cleggUrl"
Write-Output $(if ($cleggSensorIP) { "Clegg receiver: $cleggSensorIP" } else { 'Clegg receiver: automatic discovery (waiting for Nano)' })
if ($cleggReuse) { Write-Output 'Existing matching station reused.' }
else { Write-Output "Station logs: $cleggLogPath" }
if (-not $NoPlot) {
    & (Join-Path $PSScriptRoot 'start_clegg_plot.ps1') -StationUrl "http://127.0.0.1:$Port"
}
