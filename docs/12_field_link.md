# 공유기·AX210 핵심

기록 기준: 2026-10-08. 연결 명령은 [README](../README.md)에 있다.

| 항목 | 설정 / 확인 |
|---|---|
| 공유기 | ipTIME AX3000Q, 관리 주소 `192.168.0.1` |
| 노트북 | USB Ethernet → **LAN** 포트, DHCP `.100–.199` |
| Jetson | `rover-5G`, 프로필 `rover-field`, 고정 `.200/24` |
| 무선 | 사용자 선택 기준: channel 36 / 160 MHz, WPA2PSK + AES |
| 자동 연결 | 켜짐, 우선순위 10, Wi-Fi 절전 꺼짐 |

집 Wi-Fi가 이미 연결되어 있으면 자동으로 갈아타지 않는다.
`sudo nmcli connection up rover-field`로 전환한다. 암호는 개인 `.local.md`에만 보관한다.
DFS 대기가 끝난 상태에서도 160 MHz 설정은 유지되며, 더 나은 거리 성능이 측정된 것은 아니다.
채널·폭 변경은 같은 조건에서 비교하고 경기장 지정 채널이 있으면 그에 맞춘다.

## AX210 설치 기록

JetPack 6.2.1 / L4T R36.4.4 / `5.15.148-tegra`에 맞춰 NVIDIA 원본에서
`iwlwifi.ko`, `iwlmvm.ko` 두 모듈만 빌드했다. 기존 `cfg80211`, `mac80211`, Realtek 모듈은 유지했다.

- 설치: `/lib/modules/5.15.148-tegra/extra/rover-ax210-r36.4.4/`
- Jetson 빌드·설치·제거 스크립트: `~/ax210-native-r36.4.4-1791135235640/`
- 실제 AX210 PCI: `8086:2725`; firmware `66.f1c864e0.0 ty-a0-gf-a0-66.ucode`.
- Ubuntu backport는 빌드만 하고 설치하지 않았다. 일반 DKMS 설치가 현재 절차는 아니다.
- 커널 업데이트 시 다시 빌드해야 한다. unsigned 모듈 경고가 기록되었으나 벤치 연결은 성공했다.
- 기존 5 GHz 안테나 두 개를 유지해 비교한다. 6 GHz·Bluetooth 동작은 확인하지 않았다.

읽기 전용 확인 명령:

```bash
uname -r
lspci -nnk | grep -A3 -i network
modinfo -n iwlwifi
modinfo -n iwlmvm
nmcli connection show --active
iw dev
```

## 실측과 다음 거리 확인

[AX210 60초 홈 Wi-Fi 결과](../data/ax210_bench_2026-10-08/REPORT.md):
30.4 fps / 10 Mbit/s, 최장 영상 간격 72.7 ms, ping 중간 3.3 ms / 최장 45.8 ms,
미응답 0/299, 재연결 0. AX3000Q 경로에서도 화면은 확인했지만 같은 60초 측정은 아니다.
**10 m·차폐·장시간 신뢰성, stock 대비 우위는 아직 입증되지 않았다.**

필요한 현장 측정 도구 [link_test.py](../station/link_test.py)는 유지했다.
Jetson을 README의 카메라 전용 모드로 실행하고, 조종 탭을 닫은 뒤 노트북에서 실행한다.

```powershell
python station/link_test.py 192.168.0.200 --seconds 60 --label "AX210; AX3000Q; actual distance/orientation here"
```

구성·실제 거리·가림·방향을 label에 기록한다. 도구는 drive 명령 없이 영상·telemetry·ping을
측정해 [CSV](../data/link_tests.csv)에 추가한다. PASS의 500 ms 기준은 링크 선별 기준일 뿐,
모터 정지나 화면 지연·절대 신뢰성을 보장하지 않는다. 근거리 → 10 m → 방향/가림별 반복 →
15분 지속 확인 순으로 기록한다. 전원/회복·정지 동작은 별도의 실제 검증이 필요하다.
상태와 완료 조건은 [FIELD-01/02](05_한계와_로드맵.md#field-01).
