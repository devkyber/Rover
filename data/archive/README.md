# 실험·소스 보관

오래된 분석을 실행 폴더에 섞지 않도록 원본 그대로 ZIP에 보관했다.
각 ZIP 안 `SHA256.json`은 개별 파일 해시다. 아래 튜닝 ZIP은 각각 독립적으로 열리며,
**모두 같은 폴더에 펼치면** 원래 `data/odometry_tuning_2026-10-02/` 구조가 복원된다.
큰 파일 하나 대신 GitHub에 저장 가능한 크기로 나눴다.

- [9월 로그 평가](odometry_evaluation_2026-10-01.zip)
- [오도메트리 방법 비교](odometry_reference_review_2026-10-02.zip)
- [튜닝 part 01](odometry_tuning_2026-10-02_part01.zip)
- [튜닝 part 02](odometry_tuning_2026-10-02_part02.zip)
- [튜닝 part 03](odometry_tuning_2026-10-02_part03.zip)
- [정리 전 소스](source_snapshot_2026-10-08.zip): runtime, replay/분석·진단·검증 소스, dependency 목록.
- [복원 manifest](restoration_manifest.json): 별도로 복사한 원본의 정리 전 해시.

과거 보고서의 날짜별 source hash가 재현 기준이다. 10-08 소스 snapshot은 **당시 남아 있던
working tree**이며 모든 9월·10월 실험의 정확한 revision을 대신하지 않는다.
CSV sidecar에 포함된 소스, 각 실험 manifest와 함께 확인한다.
현재 코드로 실행해 예전 결과를 덮어쓰지 않는다. 오래된 source/test는 보관물이며 자동 실행하지 않는다.

조사할 때는 저장소 밖 임시 폴더에 source ZIP과 필요한 실험 ZIP을 풀고,
`logs/` 원본을 복사한다. 원문 보고서도 필요하면 [문서 ZIP](../../docs/archive/references_2026-10-08.zip)을
같은 구조로 펼친다. README의 현장 Python 환경에 분석용 패키지를 추가할 필요는 없다.
