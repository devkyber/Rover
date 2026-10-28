param([switch] $ClientsOnly, [switch] $FunctionsOnly)
# Reference implementation for Windows PowerShell 5.1, not PowerShell 7.
# Defining these functions changes no network configuration. Explicitly calling
# Start-CleggHotspotSession starts a hotspot using nonpersistent session settings.
# Supply secrets as in-memory variables; never print the passphrase or AP object.

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Runtime.WindowsRuntime
$null = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringSessionAccessPointConfiguration,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.TetheringWiFiBand,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.TetheringWiFiAuthenticationKind,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]

$script:CleggAsTaskOperation = [System.WindowsRuntimeSystemExtensions].GetMethods() |
    Where-Object {
        $_.Name -eq 'AsTask' -and $_.IsGenericMethod -and
        $_.GetGenericArguments().Count -eq 1 -and
        $_.GetParameters().Count -eq 1 -and
        $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1'
    } | Select-Object -First 1

function Wait-CleggWinRtOperation($Operation, [type] $ResultType) {
    $cleggTask = $script:CleggAsTaskOperation.MakeGenericMethod($ResultType).Invoke($null, @($Operation))
    if (-not $cleggTask.Wait(20000)) {
        throw 'WinRT operation timed out after 20 seconds; inspect the actual hotspot state before retrying.'
    }
    return $cleggTask.GetAwaiter().GetResult()
}

function Get-CleggTetheringManager {
    $cleggInternetProfile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
    if ($null -eq $cleggInternetProfile) { throw 'No internet connection profile is available.' }
    return [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($cleggInternetProfile)
}

function Start-CleggHotspotSession([string] $Ssid, [string] $Passphrase) {
    if ($Ssid.Length -lt 1 -or $Ssid.Length -gt 32) { throw 'Invalid hotspot SSID length.' }
    if ($Passphrase.Length -lt 8 -or $Passphrase.Length -gt 63 -or $Passphrase -match '[^\x20-\x7e]') {
        throw 'Hotspot passphrase must contain 8 to 63 printable ASCII characters.'
    }
    $cleggSession = New-Object Windows.Networking.NetworkOperators.NetworkOperatorTetheringSessionAccessPointConfiguration
    $cleggBand = [Windows.Networking.NetworkOperators.TetheringWiFiBand]::TwoPointFourGigahertz
    $cleggAuth = [Windows.Networking.NetworkOperators.TetheringWiFiAuthenticationKind]::Wpa2
    if (-not $cleggSession.IsBandSupported($cleggBand)) { throw '2.4 GHz hotspot is unsupported.' }
    if (-not $cleggSession.IsAuthenticationKindSupported($cleggAuth)) { throw 'WPA2 hotspot is unsupported.' }
    $cleggSession.Ssid = $Ssid
    $cleggSession.Passphrase = $Passphrase
    $cleggSession.Band = $cleggBand
    $cleggSession.AuthenticationKind = $cleggAuth
    $cleggTetheringManager = Get-CleggTetheringManager
    if ($cleggTetheringManager.TetheringOperationalState.ToString() -ne 'Off') {
        throw 'A hotspot is already active or transitioning; preserve it and inspect its state.'
    }
    $cleggResult = Wait-CleggWinRtOperation ($cleggTetheringManager.StartTetheringAsync($cleggSession)) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
    # Safe output: no credentials or complete AP configuration objects.
    [pscustomobject]@{
        Status = $cleggResult.Status.ToString()
        AdditionalErrorMessage = $cleggResult.AdditionalErrorMessage
        OperationalState = $cleggTetheringManager.TetheringOperationalState.ToString()
        RequestedBand = $cleggBand.ToString()
        RequestedAuthenticationKind = $cleggAuth.ToString()
        PersistentConfigurationModified = $false
    }
}

function Get-CleggHotspotClients {
    $cleggTetheringManager = Get-CleggTetheringManager
    foreach ($cleggClient in $cleggTetheringManager.GetTetheringClients()) {
        $cleggNames = @($cleggClient.HostNames | ForEach-Object { $_.CanonicalName })
        $cleggIPs = @($cleggNames | Where-Object {
            $cleggParsedIP = $null
            [System.Net.IPAddress]::TryParse($_, [ref] $cleggParsedIP)
        })
        [pscustomobject]@{
            MacAddress = $cleggClient.MacAddress
            HostNames = $cleggNames
            IpAddresses = $cleggIPs
        }
    }
}

if ($FunctionsOnly) { return }
if ($ClientsOnly) {
    Get-CleggHotspotClients | ConvertTo-Json -Depth 5
    exit
}
$cleggRepoRoot = Split-Path -Parent $PSScriptRoot
$cleggCredentialPath = Join-Path $cleggRepoRoot 'clegg_wifi.local.md'
$cleggCredentialText = [IO.File]::ReadAllText($cleggCredentialPath)
$cleggSsid = [regex]::Match($cleggCredentialText, '(?m)^SSID: (.+)$').Groups[1].Value.Trim()
$cleggSecret = [regex]::Match($cleggCredentialText, '(?m)^Password: (.+)$').Groups[1].Value.Trim()
$cleggResult = Start-CleggHotspotSession -Ssid $cleggSsid -Passphrase $cleggSecret
$cleggReceiptDir = Join-Path $cleggRepoRoot 'logs/clegg_station'
New-Item -ItemType Directory -Path $cleggReceiptDir -Force | Out-Null
$cleggReceiptName = 'hotspot_' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffffffZ') + '_' + [guid]::NewGuid().ToString('N') + '.json'
$cleggResult | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $cleggReceiptDir $cleggReceiptName)
$cleggResult | Format-List
