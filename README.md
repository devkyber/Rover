# Rover — 운전·재개 안내

실제 차량의 소스·설정·주행/점프 기록을 남긴 2026-10-28 KST 보관본이다.
[문서 안내](docs/README.md) · [파일·주행·영상 보고서](data/visual_report_2026-10-28/report.html) · [최종 식별](CONTEST_RELEASE.json)
설치 근거는 [최종 인계](data/final_vehicle_2026-10-28/README.md), 미완료 실물 검증은 [한계](docs/05_한계와_로드맵.md)를 따른다.

## 연결·시작

노트북 USB Ethernet은 공유기 **LAN**에 연결한다. Jetson·Scout는 같은 LAN을 쓴다. 인터넷은 필요 없다.

| 대상 | 주소 |
|---|---|
| 공유기 | http://192.168.0.1 |
| Jetson | `192.168.0.200` |
| 노트북 dashboard | http://localhost:8766/?rover=192.168.0.200 |

PowerShell에서 접속한다.

```powershell
ssh taeholee@192.168.0.200
```

Jetson에서 기존 실행을 확인한다. 실행 중이면 중복 시작하지 않는다. 없을 때만 service를 시작한다.

```bash
pgrep -af '[r]over_main.py'
sudo systemctl start rover-field.service
```

노트북의 새 PowerShell에서 실행하고 Chrome/Edge로 dashboard를 연다.

```powershell
cd C:\Users\tae06\CODE\Rover
python station/station.py --rover 192.168.0.200 --port 8766
```

`ROVER` 주소 확인 → `CONNECT`. 업데이트 뒤에는 새로고침한다. 시작·`NEW RUN` 뒤 약 5초 정지해 IMU를 정렬한다.
IMU 수신·복구는 주행과 독립적이다. 보관된 자세·위치만으로 새 데이터나 정확한 위치를 판단하지 않는다.

## 주행·화면

| 조작 | 기능 |
|---|---|
| `DRIVE`, `W / S`, `A / D` | 주행·회전, 놓으면 정지 |
| `↑ / ↓`, `SPEED LIMIT` | 속도 제한, 낮게 시작 |
| `Space`, `E-STOP` | Rover·두 Scout 중단. Rover 재개는 `RELEASE` |
| `PAN ◀ / ▶`, `← / →` (`Q / E`) | 누르는 동안 카메라 PAN |
| `RUN` 이름 → `NEW RUN` | 새 기록·IMU 재정렬 |
| `STOP LOG` | 세션·IMU 기록 종료, 주행 정지는 아님 |

Gamepad도 같은 입력을 쓴다. 재접속·E-STOP 뒤에는 새 입력으로 조작한다. `READY`는 바퀴 정지 표시다.
`EXPAND`·`RESTORE`·`FULL SCREEN`은 영상 표시를 바꾸며 `Esc`로 닫거나 복원한다.
`FILTER CHECK`·`TELEMETRY / SYSTEM LOG`에서 수신·표본 나이·오류를 확인한다.
PAN은 실제 각도 피드백·자동 끝단 제한이 없다. `SET 0`·누적 시간은 수동 참고이며 케이블 장력을 확인한다.
`LOW BATTERY`는 교체·충전 안내다. 모터 잠금·개별 셀 보호·전원 차단 기능은 없다.

## Clegg·실린더

하중을 지지하고 `UP`/`DOWN`을 누르면 실린더가 움직이며 놓으면 torque ON `HOLD`한다.
표시 기준은 자동 끝단 제한이 아니다. `TORQUE OFF`·E-STOP·만료·종료는 토크를 풀 수 있다.
`WIND`는 누르는 동안, `ARM`/`RELEASE`는 각 설정 시간 동안 동작한다. 선행 순서는 없다.
`STRING SETTINGS`의 PWM·Duration은 다음 입력부터 적용되며 시간 완료는 실제 위치 확인이 아니다.

Nano 이미지·2.4 GHz 전용 핫스팟·ignored `clegg_wifi.local.md`를 준비하고 실행한다.

```powershell
powershell.exe -NoProfile -File tools/start_clegg_station.ps1
```

파형 창을 닫아도 수신은 계속된다. 다시 열려면 `tools/start_clegg_plot.ps1`을 실행한다.
nominal g·peak·명령 요청은 실제 접촉/CIV/CBR가 아니다. 연결·보정은 [하드웨어](docs/06_하드웨어.md)를 따른다.

## Scout·지도

패널의 IP·MAC·실물을 대조한다. `LOCK`·`LAUNCH`·`PULL`은 서보 위치와 DC 상태를 독립적으로 유지한다.
`COMPRESS`·`DECOMPRESS`는 버튼을 놓아도 회전한다. counts·시간/stall 자동 정지가 없으므로 `STOP ROTATION`으로 멈춘다.
`E-STOP`은 DC·서보 모두 끈다. 창 전환·페이지 닫기는 Scout 정지 조건이 아니며 명시적 `DISCONNECT`·실제 단절·서버 종료는 정지 경로다.
`LAUNCH`는 지원 응답이 필요하다. ACK·encoder 변화는 실제 점프·거리·힘의 확인이 아니다.
발사 뒤 그래프는 자동으로 열지 않는다. `Plot viewer`는 저장 결과를 수동으로 읽는다.
`MISSION MAP`에서 기준점을 등록하고 첫 접촉·최종 정지·Clegg 타격을 구분한다. 모르는 위치는 `Unknown`으로 둔다.
사건·좌표·원본 해석은 [차량 데이터 계약](docs/13_scout_integration.md)을 따른다.

## 단절·종료·기록

입력을 놓고 `E-STOP`을 요청한 뒤 실제 정지를 확인한다. 불통이면 요청도 전달되지 않을 수 있다.
계속 움직이면 하중을 지지하고 구동 전원을 차단한다. 소프트웨어 E-STOP은 전원 차단이 아니다.
정지 상태에서 자동 재접속을 기다리고 Ethernet/LAN·공유기 `.1`·Jetson `.200`을 확인한다.
폭 변경·원복·Jetson 로컬 복구는 [설치·통신 안내](docs/11_jetson_bringup.md#네트워크와-복구)를 따른다.

정지 → 영상 `STOP & SAVE`·`SAVED` 확인 → `STOP LOG` → Jetson에서 종료한다.

```bash
sudo systemctl stop rover-field.service
sudo shutdown -h now
```

Station은 실행 창 `Ctrl+C`로 종료한다. 영상은 자동 시작하며 `STOP LOG`·`NEW RUN`·화면 단절과 독립적이다.
다시 녹화하려면 `RECORD`를 누른다. `GAPS`·`VIDEO SAVE ERROR`·`REC ERROR`는 누락/실패다.
원본은 Jetson의 `logs/`에 저장하며 노트북에 자동 복사하지 않는다. 실제 동작의 관련 폴더·metadata·source/config를 함께 복사한다.
기존 주행·점프는 [logs](logs/README.md), 결과·방법은 [data](data/README.md), 환경 복원은 [설치 안내](docs/11_jetson_bringup.md)에 있다.
