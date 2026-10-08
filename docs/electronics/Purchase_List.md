# 장비·구매 기록

2026-10-07 구매 기록과 10-08 설치 확인의 요약. “Recorded bought”는 구매 기록이며
실물 도착·현재 수량을 모두 확인했다는 뜻은 아니다. PCB 부품은 [현재 BOM 원본](PDB_Design_Handover.md)을 따른다.

| Item | Recorded status / quantity | Role or evidence |
|---|---|---|
| Jetson Orin Nano Super | Owned, 1 | [Installed and exercised](../11_jetson_bringup.md) |
| DYNAMIXEL XH430-V350-R | Owned, 5 | Four wheels and planned hammer; wheel bus recordings are in [logs](../../logs/README.md) |
| FEETECH FS90MGR | Owned, 3 | Two lift servos and camera pan; installed-unit calibration belongs to HW-01 |
| Arducam B0201 | Owned, 1 | [Camera integration record](../11_jetson_bringup.md) |
| ROBOTIS U2D2 | Used in earlier bus tests | Confirm the unit/harness on the assembled rover during HW-01 |
| U2D2 Power Hub | Recorded owned, 1 | Retained as bench spare; current power distribution comes from the PDB design |
| 6S 2,800 mAh LiPo | Recorded bought, 1 | Runtime and warning calibration require measurements on this pack |
| Intel AX210NGW and antennas | Installed; bench connection verified | Field adoption: [FIELD-02](../05_한계와_로드맵.md#field-02) |
| 256 GB NVMe SSD | Bought and installed, 1 | Jetson boot storage |
| ipTIME AX3000Q | Bought and configured, 1 | [Router record](../12_field_link.md) |
| Bosch Shuttle Board 3.0 BMI088 | Recorded bought; spare not confirmed | Intended replacement IMU; [IMU-01](../05_한계와_로드맵.md#imu-01) |
| Raspberry Pi Pico 2 H | Recorded bought | Servo/battery I/O board |
| ipTIME U1G-C | Recorded bought | Laptop-to-router Ethernet |
| CAT.6 cable, 15–20 m | Recorded bought | Router placement flexibility |
| USB-A to micro-USB data cable | Recorded bought; final quantity/fit unconfirmed | Pico and U2D2 USB |
| INA226 modules | Recorded bought, 4 | Bench/fallback units; read the actual shunt marking and configure it accordingly |
| 5.5×2.5 mm DC plug cable | Recorded bought | Jetson power harness; match the current board output |
| ADXL375 | Order/arrival unconfirmed in the record | Hammer impact acquisition; [MISSION-01](../05_한계와_로드맵.md#mission-01) |


배선·충전기·공구·예비품의 실제 수량은 조립 때 확인한다. 옛 가격·재고는 현재 구매 근거가 아니다.
이전 선택 근거는 [문서 아카이브](../archive/README.md)에 있다.
