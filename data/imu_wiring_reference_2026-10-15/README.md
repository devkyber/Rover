# BMI088 배선 참고 자료

실제 연결표는 [하드웨어 안내](../../docs/06_하드웨어.md#bmi088-연결)에만 둔다.
여기는 그 연결을 다시 확인할 때 필요한 원본과 최종 그림이다.

| 자료 | 용도 |
|---|---|
| [Jetson–BMI088 연결 그림](jetson-orin-nano-super-bmi088-wiring.png) | 커넥터와 전원·SPI 위치 |
| [P1/P2 방향 상세](bmi088-p1-p2-rotated-left.png) | 두 헤더의 핀 번호 방향 |
| [Bosch datasheet, rev. 1.9](sources/bosch-bmi088-datasheet-rev1.9.pdf) | 전압·SPI 신호·칩 pad |
| [Bosch Shuttle Board 3.0 flyer, v1.1](sources/bst-bmi088-sf000-v1.1.pdf) | 보드 식별과 P1/P2 회로·핀 표 |
| [NVIDIA carrier specification, v1.3](sources/nvidia-carrier-spec-SP-11324-001-v1.3.pdf) | Jetson J12 헤더·신호 전압 |
| [Developer kit 참고 사진](sources/nvidia-devkit-reference.png) | 개발 보드 형태와 J12 위치 |
| [사용자 제공 Shuttle 도면](sources/user-shuttle-outline.png) | 상세 그림의 방향 기준 |

두 최종 배선 PNG는 제조사 자료를 참조한 생성 일러스트다. 실물 배선 사진·실측 도면이 아니다.
P1/P2 상세는 사용자가 지정한 반시계 90° 방향을 따른다. 실물의 핀 1 표시를 먼저 확인한다.

제조사 출처: [Bosch datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf),
[Bosch Shuttle flyer](https://www.bosch-sensortec.com/media/boschsensortec/downloads/shuttle_board_flyer/application_board_3_1/bst-bmi088-sf000.pdf),
[NVIDIA Download Center](https://developer.nvidia.com/embedded/downloads),
[NVIDIA hardware layout](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html).
원본 문서의 권리는 각 제조사에 있다. 중복 렌더 이미지와 그림 제작 프롬프트는 정리했다.

## 보존 파일 SHA256

- `bmi088-p1-p2-rotated-left.png`: `1f14b3a61b48c7c474b096124a4a863b0058f3d992eeb9b229842a41d51c65d6`
- `jetson-orin-nano-super-bmi088-wiring.png`: `010cc7019b3c1a3b82ae1cd6cd0ea150f412519f0dfae86f6199a2ecb9b1b01c`
- `sources/bosch-bmi088-datasheet-rev1.9.pdf`: `53ab734dba49ac202fa6182fc2b545df6c6d7459c9fec6e851a1209e6bac417e`
- `sources/bst-bmi088-sf000-v1.1.pdf`: `5824aab1e4f5c5638449c13c76d17453b047838e4b216fc9b79a6450b1e0de24`
- `sources/nvidia-carrier-spec-SP-11324-001-v1.3.pdf`: `4a0f7ba948bce4881e176f0f8636ef3dbd40e3df9dd33134e6a7433359d18c02`
- `sources/nvidia-devkit-reference.png`: `a961cccb670e025ee1eb35cd01f9331e72d3ea1b6d5d57f69a52f2faf5318a50`
- `sources/user-shuttle-outline.png`: `54db9a7bcbac413a2321dfbdb419205ad741b2b98203fcf6b632ff34a807d2e1`
