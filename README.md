# vehicle_painting_robot

차량 부품(후드·트렁크·펜더·범퍼) 도장 셀: 부품 점군 → 도장 경로 → 8축(갠트리 2 + Doosan A0912 6) 모션 → RViz 검증.

| 디렉터리 | 역할 | 문서 |
|---|---|---|
| `painting_trajectory_planner/` | 점군에서 row 별 TCP 도장 경로 CSV 생성 (`path_planner/test/csv/<part>/`) | `painting_trajectory_planner/README.md` |
| `workstation/` | 스캔 → 경로 생성 → 코어 포인트 수정 데스크톱 앱 (Tauri) | `workstation/README.md` |
| `robot_control/` | Doosan A0912 제어 스크립트, URDF, 8축 NLP 솔버 `trajectory_nlp.py` | `robot_control/README.md` |
| `motion_planning/` | planner CSV → 8축 관절 궤적(NLP) → row 전이 → 충돌 검사 → 툴체인저 통합 시퀀스 → RViz. ROS2 패키지 `paint_viz`, 배치 스크립트 `nlp4cm`, conda 환경 | **`motion_planning/README.md`** |

두 파이프라인의 접점은 **`painting_trajectory_planner/path_planner/test/csv/<part>/trajectory_NNN.csv`** 하나다.
planner 가 거기에 쓰고, `motion_planning/nlp4cm/run_all.sh` 가 거기서 읽는다 (시간 프로파일을 붙여 `nlp4cm/input/<part>` 로).
파츠 이름은 디렉터리 이름이다.

## 빠른 시작 (모션/RViz)

```bash
bash motion_planning/setup_workspace.sh                 # 최초 1회: conda env(paintcell) + ~/paint_cell_ws 빌드 (ROS2 Humble 필요)
source motion_planning/activate_env.sh                  # 매 셸
motion_planning/nlp4cm/viz/play.sh ES300h_hood 2.0      # 저장소에 든 결과를 RViz 로 재생
motion_planning/nlp4cm/run_all.sh                       # planner CSV 가 바뀌었을 때 전체 재계산 (NLP + 전이)
motion_planning/nlp4cm/sequence_all.sh                  # 툴체인저 포함 통합 시퀀스
```

자세한 디렉터리 규칙, CSV 스키마, 환경 설정, 현재 결과는 `motion_planning/README.md`.
