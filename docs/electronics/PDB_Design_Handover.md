# PDB 원본 위치

현재 회로·BOM·커넥터·제작 검증의 원본은 형제 저장소의
[PCB/projects/rover-pdb/README.md](../../../PCB/projects/rover-pdb/README.md)다.
이 문서는 별도 회로 사양이 아니다. Rover만 받은 경우 보드 담당자에게 해당 revision을 받는다.

2026-10-07 소유자는 설계 완료를 보고했다. 조립·부하·실차 검증 완료를 뜻하지 않는다.
오래된 LM73606 설계의 pinout/BOM을 현재 보드에 적용하지 않는다.

Rover 쪽 원본: [config.py](../../rover/config.py), [Pico 펌웨어](../../firmware/pico/main.py),
[udev](../../jetson/99-rover-serial.rules). 보드 외 장비는 [구매 목록](Purchase_List.md).
통합 검증은 [HW-01](../05_한계와_로드맵.md#hw-01).
