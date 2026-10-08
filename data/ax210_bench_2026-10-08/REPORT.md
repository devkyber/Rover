# AX210 벤치 — 2026-10-08

**기본 동작 확인, 10 m 신뢰성은 미검증.** JetPack 6.2.1, kernel `5.15.148-tegra`,
AX210 PCI `8086:2725`, firmware `66.f1c864e0.0 ty-a0-gf-a0-66.ucode`.
홈 Wi-Fi channel 36 / 80 MHz, 양쪽 무선. 거리·방향·안테나 조립은 미확인.
모터/Pico off, fake IMU, 1080p H.264 카메라만 사용했다.

| 60초 결과 | 값 |
|---|---:|
| 영상 | 30.4 fps / 10.0 Mbit/s |
| 영상 간격 p99 / 최대 | 63.1 / 72.7 ms |
| application ping 중간 / p99 / 최대 | 3.3 / 40.6 / 45.8 ms |
| ping 미응답 / 재연결 | 0/299 / 0 |
| telemetry | 20.0 Hz |
| RSSI 중간 / 최저 | −48 / −49 dBm |

근거: [사전 상태·소스 해시](preflight.json), [실행 명령](runtime.json),
[측정 stdout](link_60s.txt), [원본 CSV](../link_tests.csv), [사후 상태](postflight.json).
별도 idle ICMP는 [100/100 응답](icmp_idle.json), 평균 3.73 ms·최대 19 ms였다.
ICMP와 application ping은 다른 측정이다. dirty 소스이므로 Git HEAD만으로 재현되지 않는다.

이후 [AX3000Q 연결](field_association.json): `rover-5G`, `.200`, channel 36 / 160 MHz.
노트북 케이블을 WAN에서 LAN으로 옮긴 뒤 `.100`을 받고 영상·telemetry가 표시됐다.
[홈 화면](station_live.png), [AX3000Q 화면](station_ax3000q.png)은 순간 관찰이며 60초 평균이 아니다.

USB NCM 오류는 후속에도 있었고 복구는 미검증. stock과 위치·부하를 맞춘 비교가 아니므로
AX210 우위를 주장할 수 없다. frame 도착 간격은 화면 지연이 아니다.
2.4/6 GHz·Bluetooth·반복 부팅·AP 단절·장시간·10 m 차폐 시험은 미실시.
[FIELD-01/02](../../docs/05_한계와_로드맵.md#field-01)에서 추적한다.
전체 원문은 [문서 보관](../../docs/archive/README.md)에 유지했다.
