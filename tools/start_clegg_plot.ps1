#Requires -Version 5.1
param([string] $StationUrl = 'http://127.0.0.1:8766')
$ErrorActionPreference = 'Stop'
$cleggPlotRoot = [IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$cleggPlotScript = Join-Path $PSScriptRoot 'plot_clegg.py'
$cleggPlotPython = Join-Path $cleggPlotRoot '.venv\Scripts\pythonw.exe'
if (-not (Test-Path -LiteralPath $cleggPlotPython)) {
    $cleggPlotPython = Join-Path $cleggPlotRoot '.venv\Scripts\python.exe'
}
foreach ($cleggPlotRequired in @($cleggPlotScript, $cleggPlotPython)) {
    if (-not (Test-Path -LiteralPath $cleggPlotRequired -PathType Leaf)) { throw "Required file missing: $cleggPlotRequired" }
}
$cleggPlotUri = [Uri] $StationUrl
if (-not $cleggPlotUri.IsAbsoluteUri -or $cleggPlotUri.Scheme -ne 'http' -or
    $cleggPlotUri.Host -notin @('127.0.0.1', 'localhost', '[::1]', '::1') -or
    $cleggPlotUri.UserInfo -or $cleggPlotUri.Query -or $cleggPlotUri.Fragment -or $cleggPlotUri.AbsolutePath -ne '/') {
    throw 'StationUrl must be a local receiver HTTP address, without a path, credentials or query.'
}
$cleggPlotAddress = $cleggPlotUri.GetLeftPart([UriPartial]::Authority)

if (-not ('CleggPlotArgv' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class CleggPlotArgv {
    [DllImport("shell32.dll", SetLastError=true)]
    static extern IntPtr CommandLineToArgvW([MarshalAs(UnmanagedType.LPWStr)] string command, out int count);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr pointer);
    public static string[] Parse(string command) {
        int count; IntPtr memory = CommandLineToArgvW(command, out count);
        if (memory == IntPtr.Zero) return new string[0];
        try {
            string[] result = new string[count];
            for (int i=0; i<count; i++) result[i] = Marshal.PtrToStringUni(Marshal.ReadIntPtr(memory, i*IntPtr.Size));
            return result;
        } finally { LocalFree(memory); }
    }
}
'@
}
foreach ($cleggPlotProcess in @(Get-CimInstance Win32_Process -Filter "Name LIKE 'python%exe'")) {
    if (-not $cleggPlotProcess.CommandLine) { continue }
    $cleggPlotArgv = [CleggPlotArgv]::Parse($cleggPlotProcess.CommandLine)
    if ($cleggPlotArgv.Length -eq 4 -and
        [string]::Equals($cleggPlotArgv[0], $cleggPlotPython, [StringComparison]::OrdinalIgnoreCase) -and
        [string]::Equals($cleggPlotArgv[1], $cleggPlotScript, [StringComparison]::OrdinalIgnoreCase) -and
        $cleggPlotArgv[2] -eq '--station' -and $cleggPlotArgv[3] -eq $cleggPlotAddress) {
        Write-Output "Matplotlib watcher already running: PID $($cleggPlotProcess.ProcessId). A new capture opens the plot."
        exit 0
    }
}
$cleggPlotLog = Join-Path $cleggPlotRoot ('logs\clegg_plot\' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmss_fffffffZ') + '_' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
$null = New-Item -ItemType Directory -Path $cleggPlotLog
$cleggPlotStarted = Start-Process -FilePath $cleggPlotPython -ArgumentList @(('"' + $cleggPlotScript + '"'), '--station', $cleggPlotAddress) -WorkingDirectory $cleggPlotRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $cleggPlotLog 'stdout.log') -RedirectStandardError (Join-Path $cleggPlotLog 'stderr.log')
[pscustomobject]@{
    CreatedUtc = [DateTime]::UtcNow.ToString('o'); ProcessId = $cleggPlotStarted.Id
    Script = $cleggPlotScript; Python = $cleggPlotPython; Station = $cleggPlotAddress
    SourceSHA256 = (Get-FileHash -LiteralPath $cleggPlotScript -Algorithm SHA256).Hash.ToLowerInvariant()
    Behavior = 'Native Matplotlib window on a new completed impact; historical capture at startup is skipped.'
    SensorConnection = 'None: local receiver HTTP GET only'
} | ConvertTo-Json | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $cleggPlotLog 'launch.json')
Start-Sleep -Milliseconds 800
$cleggPlotStarted.Refresh()
if ($cleggPlotStarted.HasExited -and $cleggPlotStarted.ExitCode -ne 0) {
    throw "Matplotlib watcher exited; inspect $cleggPlotLog"
}
Write-Output "Matplotlib watcher started. A new capture opens the plot. Logs: $cleggPlotLog"
