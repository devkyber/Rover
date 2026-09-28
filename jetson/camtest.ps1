# 젯슨 카메라를 한 번에 검증한다. 스트리밍을 쓰지 않으므로 USB 링크가
# 불안정해도 결국 성공한다 (젯슨에서 녹화 -> 파일만 재시도하며 내려받기).
#
#   .\camtest.ps1              # 6초, 컨택트시트 + 원본 1장
#   .\camtest.ps1 -Secs 15 -H264   # H264 클립까지
param(
    [int]$Secs = 6,
    [switch]$H264,
    [string]$Jetson = "kit@192.168.55.1"
)

$key = "$env:USERPROFILE\.ssh\id_ed25519_jetson"
$sshExe = "$env:WINDIR\System32\OpenSSH\ssh.exe"
$scpExe = "$env:WINDIR\System32\OpenSSH\scp.exe"
$opts = @("-i", $key, "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=no",
          "-o", "UserKnownHostsFile=NUL", "-o", "ConnectTimeout=10", "-o", "BatchMode=yes", "-o", "LogLevel=ERROR")

function Invoke-Jetson([string]$cmd) {
    foreach ($i in 1..8) {
        $out = & $sshExe @opts $Jetson $cmd
        if ($LASTEXITCODE -eq 0) { return $out }
        Start-Sleep -Milliseconds 800; Write-Host "  재시도 $i..." -ForegroundColor DarkGray
    }
    throw "젯슨 접속 실패 (8회). USB 케이블과 ip 192.168.55.1 확인."
}

function Copy-FromJetson([string]$remote, [string]$local) {
    foreach ($i in 1..8) {
        & $scpExe @opts "${Jetson}:$remote" $local
        if ($LASTEXITCODE -eq 0) { return $true }
        Start-Sleep -Milliseconds 800; Write-Host "  재시도 $i..." -ForegroundColor DarkGray
    }
    return $false
}

Write-Host "1. 카메라를 잡고 있는 프로세스 정리" -ForegroundColor Cyan
Invoke-Jetson "pkill -f cam_stream.py || true; rm -rf ~/camtest ~/camtest.log; echo ok" | Out-Null

Write-Host "2. 젯슨에서 ${Secs}초 녹화" -ForegroundColor Cyan
# 링크가 끊겨도 녹화는 계속되도록 detach 시킨다
Invoke-Jetson "nohup python3 ~/cam_burst.py --secs $Secs > ~/camtest.log 2>&1 & disown; echo ok" | Out-Null
Start-Sleep -Seconds ($Secs + 12)

foreach ($i in 1..10) {
    $log = Invoke-Jetson "cat ~/camtest.log 2>/dev/null"
    if ($log -match "저장:") { break }
    Start-Sleep -Seconds 4
}
$log

if ($H264) {
    Write-Host "3. H264 하드웨어 인코딩 녹화" -ForegroundColor Cyan
    Invoke-Jetson "nohup ~/rec_h264.sh $Secs > ~/h264.log 2>&1 & disown; echo ok" | Out-Null
    Start-Sleep -Seconds ($Secs + 15)
    Invoke-Jetson "tail -2 ~/h264.log"
}

Write-Host "4. 결과 내려받기" -ForegroundColor Cyan
foreach ($f in @("sheet.jpg", "full.jpg")) {
    if (Copy-FromJetson "~/camtest/$f" ".\$f") { Write-Host "  $f" }
    else { Write-Host "  $f 실패" -ForegroundColor Red }
}
if ($H264) {
    if (Copy-FromJetson "~/camtest/clip.mkv" ".\clip.mkv") { Write-Host "  clip.mkv" }
}

Write-Host "`n완료. sheet.jpg 를 열어 9칸이 모두 정상인지 보세요." -ForegroundColor Green
